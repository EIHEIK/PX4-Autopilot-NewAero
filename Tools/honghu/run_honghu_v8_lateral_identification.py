#!/usr/bin/env python3
"""Historical airborne roll-step identification for the Honghu Wing V8.

The test deliberately bypasses runway takeoff and mission guidance.  Gazebo
spawns the aircraft high above the ground, a passive diagnostic hook supplies
one initial forward-velocity sample, and MAVLink offboard attitude setpoints
exercise the PX4 roll attitude/rate loops.  SITL parameter and mission files
are restored byte-for-byte after every run.

The initial-velocity hook used to produce the archived 0.5 ms reports is not
present in the current generated V8 model.  Until that hook is deliberately
restored or replaced, use run_honghu_v8_dynamic_acceptance.py route for current
lateral verification; this script is retained to document the old experiment.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import statistics
import subprocess
import tempfile
import time
from pathlib import Path

from pymavlink import mavutil


ROOT = Path(__file__).resolve().parents[2]
PX4_BIN = ROOT / "build/px4_sitl_default/bin"
ROOTFS = ROOT / "build/px4_sitl_default/rootfs"
TEST_PORT = 15550
STATE_FILES = (
    ROOTFS / "parameters.bson",
    ROOTFS / "parameters_backup.bson",
    ROOTFS / "dataman",
)
# Each tuple is (commanded roll [deg], duration [s]).  The zero-roll plateaus
# make positive and negative steps independently measurable.
ROLL_SCHEDULE = (
    (0.0, 5.0),
    (10.0, 7.0),
    (0.0, 7.0),
    (-10.0, 7.0),
    (0.0, 7.0),
    (20.0, 8.0),
    (0.0, 8.0),
    (-20.0, 8.0),
    (0.0, 8.0),
)


def run(command: list[str], timeout: float = 15.0, check: bool = True) -> str:
    result = subprocess.run(
        command,
        cwd=ROOT,
        check=check,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
    )
    return result.stdout


class SitlStateSnapshot:
    def __init__(self) -> None:
        self.snapshot: dict[Path, tuple[bool, bytes]] = {}

    def capture(self) -> None:
        self.snapshot = {
            path: (path.exists(), path.read_bytes() if path.exists() else b"")
            for path in STATE_FILES
        }

    def restore(self) -> None:
        for path, (existed, payload) in self.snapshot.items():
            if existed:
                path.parent.mkdir(parents=True, exist_ok=True)
                with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as temporary:
                    temporary.write(payload)
                    temporary_path = Path(temporary.name)
                temporary_path.replace(path)
            elif path.exists():
                path.unlink()


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> list[float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def wrap_degrees(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


class LateralIdentificationRun:
    def __init__(
        self,
        step_size: float,
        parameter_overrides: dict[str, float],
        pitch_deg: float,
        cruise_thrust: float,
    ) -> None:
        self.step_size = step_size
        self.parameter_overrides = parameter_overrides
        self.pitch_deg = pitch_deg
        self.cruise_thrust = cruise_thrust
        self.snapshot = SitlStateSnapshot()
        self.process: subprocess.Popen | None = None
        self.log_file = None
        self.log_path: Path | None = None
        self.connection = None
        self.target_system = 1
        self.target_component = 1

    def assert_clean_runtime(self) -> None:
        result = subprocess.run(
            ["pgrep", "-af", "px4|gz sim|gzserver|ruby.*gz"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        active = [line for line in result.stdout.splitlines() if "pgrep -af" not in line]
        if active:
            raise RuntimeError("existing PX4/Gazebo process detected:\n" + "\n".join(active))

    def start(self) -> None:
        self.assert_clean_runtime()
        self.snapshot.capture()
        output_dir = ROOT / "analysis_outputs"
        output_dir.mkdir(parents=True, exist_ok=True)
        label = f"{self.step_size * 1000:g}".replace(".", "p")
        self.log_path = output_dir / f"honghu_v8_lateral_identification_{label}ms.log"
        self.log_file = self.log_path.open("w", encoding="utf-8")
        environment = os.environ.copy()
        environment.update(
            {
                "HEADLESS": "1",
                "PX4_GZ_MAX_STEP_SIZE": f"{self.step_size:.7f}",
                "PX4_GCS_REMOTE_PORT": str(TEST_PORT),
                # Keep the diagnostic reader and PX4/Gazebo bridge within the
                # validated real-time message budget. Normal QGC runs do not
                # set this opt-in environment variable.
                "PX4_GCS_MINIMAL_STREAMS": "1",
                # High spawn altitude gives the estimator and the acceleration
                # phase ample clearance without exercising runway logic.
                "PX4_GZ_MODEL_POSE": "0,0,500,0,0,0",
                # Read only by the environment-gated V8 identification hook.
                # It is delayed until after estimator startup and offboard arm.
                "HONGHU_V8_INITIAL_VELOCITY": "40,0,0",
                "HONGHU_V8_INITIAL_VELOCITY_AT_S": "6.5",
                "HONGHU_V8_INITIAL_VELOCITY_CANARD_DEG": "3.0",
            }
        )
        self.process = subprocess.Popen(
            ["make", "px4_sitl", "gz_honghu_wing_150kg_v8"],
            cwd=ROOT,
            env=environment,
            stdout=self.log_file,
            stderr=subprocess.STDOUT,
            stdin=subprocess.PIPE,
            start_new_session=True,
        )
        self.connection = mavutil.mavlink_connection(
            f"udpin:127.0.0.1:{TEST_PORT}", source_system=250,
        )
        heartbeat = self.connection.wait_heartbeat(timeout=60)
        if heartbeat is None:
            raise TimeoutError(f"PX4 heartbeat not received; see {self.log_path}")
        self.target_system = heartbeat.get_srcSystem()
        self.target_component = heartbeat.get_srcComponent()
        self.configure_streams()
        self.set_parameters()

    def configure_streams(self) -> None:
        # The step metrics need 20 Hz attitude data; matching the aggregate
        # stream rate to the receive loop prevents stale-kernel-buffer results.
        for message_id, rate_hz in ((30, 20), (32, 10), (74, 10), (245, 2)):
            self.connection.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                1_000_000.0 / rate_hz,
                0, 0, 0, 0, 0,
            )

    def set_parameters(self) -> None:
        values = {
            "COM_RC_IN_MODE": 4,
            "COM_DISARM_LAND": -1,
            "COM_DISARM_PRFLT": -1,
            "COM_OF_LOSS_T": 5.0,
            "RWTO_TAXI_TEST": 0,
            # OFFBOARD bypasses the AUTO/CLIMBOUT state transition that
            # normally deploys the V3-compatible +4 deg cruise canard. Shift
            # the temporary neutral so this air-start test uses the same
            # airborne aerodynamic configuration. The SITL state snapshot is
            # restored byte-for-byte after the run.
            "FW_CANARD_NEUT": 0.633333,
        }
        values.update(self.parameter_overrides)
        for name, value in values.items():
            output = run([str(PX4_BIN / "px4-param"), "set", name, str(value)])
            if "ERROR" in output.upper():
                raise RuntimeError(f"failed to set {name}: {output}")

    def send_gcs_heartbeat(self) -> None:
        self.connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def send_attitude_target(
        self, roll_deg: float, pitch_deg: float, yaw_rad: float, thrust: float,
    ) -> None:
        type_mask = (
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
            | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
            | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
        )
        self.connection.mav.set_attitude_target_send(
            int(time.monotonic() * 1000.0) & 0xFFFFFFFF,
            self.target_system,
            self.target_component,
            type_mask,
            quaternion_from_euler(math.radians(roll_deg), math.radians(pitch_deg), yaw_rad),
            0.0, 0.0, 0.0,
            thrust,
        )

    def wait_for_initial_state(self) -> tuple[object, object, object]:
        deadline = time.monotonic() + 35.0
        attitude = None
        local = None
        vfr = None
        last_heartbeat = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_heartbeat > 0.8:
                self.send_gcs_heartbeat()
                last_heartbeat = now
            message = self.connection.recv_match(
                type=["ATTITUDE", "LOCAL_POSITION_NED", "VFR_HUD"],
                blocking=True,
                timeout=0.2,
            )
            if message is None:
                continue
            if message.get_type() == "ATTITUDE":
                attitude = message
            elif message.get_type() == "LOCAL_POSITION_NED":
                local = message
            else:
                vfr = message
            if attitude is not None and local is not None and vfr is not None:
                return attitude, local, vfr
        raise TimeoutError("complete initial attitude/position/airspeed state not received")

    def prime_and_arm(self, yaw_rad: float) -> None:
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            self.send_gcs_heartbeat()
            self.send_attitude_target(0.0, self.pitch_deg, yaw_rad, 1.0)
            self.connection.recv_match(blocking=True, timeout=0.04)
        run([str(PX4_BIN / "px4-commander"), "mode", "offboard"])
        run([str(PX4_BIN / "px4-commander"), "arm", "-f"])

    def accelerate_and_settle(self, yaw_rad: float) -> tuple[list[dict], object, object, object]:
        samples: list[dict] = []
        attitude, local, vfr = self.wait_for_initial_state()
        start = time.monotonic()
        initialized_at = None
        last_heartbeat = 0.0
        while time.monotonic() - start < 18.0:
            now = time.monotonic()
            if now - last_heartbeat > 0.8:
                self.send_gcs_heartbeat()
                last_heartbeat = now
            # Follow the measured yaw so the roll excitation does not command
            # an unphysical banked attitude with a frozen heading. PX4 remains
            # free to produce the coordinated-turn yaw response.
            self.send_attitude_target(
                0.0, self.pitch_deg, float(attitude.yaw), self.cruise_thrust,
            )
            message = self.connection.recv_match(
                type=["ATTITUDE", "LOCAL_POSITION_NED", "VFR_HUD"],
                blocking=True,
                timeout=0.04,
            )
            if message is not None:
                if message.get_type() == "ATTITUDE":
                    attitude = message
                elif message.get_type() == "LOCAL_POSITION_NED":
                    local = message
                else:
                    vfr = message
            horizontal_speed = math.hypot(float(local.vx), float(local.vy))
            # Do not use scalar airspeed for this decision: during the initial
            # fall it can become large even though forward speed is still low.
            if (
                initialized_at is None
                and horizontal_speed >= 35.0
                and abs(float(local.vz)) <= 8.0
            ):
                initialized_at = now
            samples.append(self.make_sample(now - start, -1, 0.0, attitude, local, vfr))
            if initialized_at is not None and now - initialized_at >= 6.0:
                return samples, attitude, local, vfr
            if abs(math.degrees(float(attitude.roll))) > 70.0:
                raise RuntimeError("aircraft exceeded 70 deg roll during acceleration")
        raise TimeoutError("failed to reach and settle near the 38 m/s test condition")

    @staticmethod
    def make_sample(
        elapsed_s: float,
        phase_index: int,
        roll_setpoint_deg: float,
        attitude: object,
        local: object,
        vfr: object,
    ) -> dict:
        return {
            "elapsed_s": elapsed_s,
            "phase_index": phase_index,
            "roll_setpoint_deg": roll_setpoint_deg,
            "roll_deg": math.degrees(float(attitude.roll)),
            "pitch_deg": math.degrees(float(attitude.pitch)),
            "yaw_deg": math.degrees(float(attitude.yaw)),
            "roll_rate_deg_s": math.degrees(float(attitude.rollspeed)),
            "airspeed_m_s": float(vfr.airspeed),
            "groundspeed_m_s": math.hypot(float(local.vx), float(local.vy)),
            "north_m": float(local.x),
            "east_m": float(local.y),
            "down_m": float(local.z),
        }

    def run_schedule(
        self, yaw_rad: float, attitude: object, local: object, vfr: object,
    ) -> list[dict]:
        samples: list[dict] = []
        start = time.monotonic()
        phase_start = start
        phase_index = 0
        last_heartbeat = 0.0
        total_duration = sum(duration for _, duration in ROLL_SCHEDULE)
        while time.monotonic() - start <= total_duration:
            now = time.monotonic()
            while (
                phase_index + 1 < len(ROLL_SCHEDULE)
                and now - phase_start >= ROLL_SCHEDULE[phase_index][1]
            ):
                phase_start += ROLL_SCHEDULE[phase_index][1]
                phase_index += 1
            roll_setpoint = ROLL_SCHEDULE[phase_index][0]
            if now - last_heartbeat > 0.8:
                self.send_gcs_heartbeat()
                last_heartbeat = now
            self.send_attitude_target(
                roll_setpoint, self.pitch_deg, float(attitude.yaw), self.cruise_thrust,
            )
            message = self.connection.recv_match(
                type=["ATTITUDE", "LOCAL_POSITION_NED", "VFR_HUD"],
                blocking=True,
                timeout=0.025,
            )
            if message is not None:
                if message.get_type() == "ATTITUDE":
                    attitude = message
                elif message.get_type() == "LOCAL_POSITION_NED":
                    local = message
                else:
                    vfr = message
            if not samples or now - start - samples[-1]["elapsed_s"] >= 0.025:
                samples.append(
                    self.make_sample(now - start, phase_index, roll_setpoint, attitude, local, vfr)
                )
            if abs(math.degrees(float(attitude.roll))) > 70.0:
                raise RuntimeError("aircraft exceeded 70 deg roll during step schedule")
        return samples

    @staticmethod
    def evaluate(samples: list[dict]) -> dict:
        phase_metrics: list[dict] = []
        for phase_index, (command, duration) in enumerate(ROLL_SCHEDULE):
            phase = [sample for sample in samples if sample["phase_index"] == phase_index]
            if not phase:
                continue
            prior = [sample for sample in samples if sample["elapsed_s"] < phase[0]["elapsed_s"]]
            initial = statistics.fmean(sample["roll_deg"] for sample in prior[-20:]) if prior else phase[0]["roll_deg"]
            final_window = [sample for sample in phase if phase[-1]["elapsed_s"] - sample["elapsed_s"] <= 1.0]
            final = statistics.fmean(sample["roll_deg"] for sample in final_window)
            requested_change = command - initial
            direction = 1.0 if requested_change >= 0.0 else -1.0

            def crossing_time(fraction: float) -> float | None:
                threshold = initial + fraction * requested_change
                for sample in phase:
                    if direction * (sample["roll_deg"] - threshold) >= 0.0:
                        return sample["elapsed_s"] - phase[0]["elapsed_s"]
                return None

            directional_peak = max(direction * sample["roll_deg"] for sample in phase) * direction
            overshoot = direction * (directional_peak - command)
            phase_metrics.append(
                {
                    "phase_index": phase_index,
                    "command_deg": command,
                    "duration_s": duration,
                    "initial_roll_deg": initial,
                    "final_roll_mean_deg": final,
                    "steady_error_deg": wrap_degrees(command - final),
                    "rise_time_63_s": crossing_time(0.63),
                    "rise_time_90_s": crossing_time(0.90),
                    "directional_peak_deg": directional_peak,
                    "overshoot_deg": max(0.0, overshoot),
                    "roll_rate_peak_abs_deg_s": max(abs(sample["roll_rate_deg_s"]) for sample in phase),
                }
            )

        altitudes = [-sample["down_m"] for sample in samples]
        return {
            "phase_metrics": phase_metrics,
            "sample_count": len(samples),
            "airspeed_min_m_s": min(sample["airspeed_m_s"] for sample in samples),
            "airspeed_max_m_s": max(sample["airspeed_m_s"] for sample in samples),
            "altitude_change_m": altitudes[-1] - altitudes[0],
            "altitude_range_m": max(altitudes) - min(altitudes),
            "roll_max_abs_deg": max(abs(sample["roll_deg"]) for sample in samples),
            "pitch_max_abs_deg": max(abs(sample["pitch_deg"]) for sample in samples),
        }

    def execute(self) -> dict:
        try:
            self.start()
            attitude, _, _ = self.wait_for_initial_state()
            yaw_rad = float(attitude.yaw)
            self.prime_and_arm(yaw_rad)
            acceleration_samples, attitude, local, vfr = self.accelerate_and_settle(yaw_rad)
            samples = self.run_schedule(yaw_rad, attitude, local, vfr)
            return {
                "scenario": "lateral_identification",
                "step_size_s": self.step_size,
                "parameter_overrides": self.parameter_overrides,
                "commanded_pitch_deg": self.pitch_deg,
                "commanded_cruise_thrust": self.cruise_thrust,
                "roll_schedule": [
                    {"roll_deg": command, "duration_s": duration}
                    for command, duration in ROLL_SCHEDULE
                ],
                "metrics": self.evaluate(samples),
                "acceleration_samples": acceleration_samples,
                "samples": samples,
                "make_log": str(self.log_path),
            }
        finally:
            self.stop()

    def stop(self) -> None:
        if self.process is not None:
            try:
                run([str(PX4_BIN / "px4-commander"), "disarm", "-f"], timeout=3.0, check=False)
                run([str(PX4_BIN / "px4-shutdown")], timeout=5.0, check=False)
            except (OSError, subprocess.SubprocessError):
                pass
            try:
                run(
                    ["gz", "service", "-s", "/server_control", "--reqtype", "gz.msgs.ServerControl",
                     "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", "stop: true"],
                    timeout=5.0,
                    check=False,
                )
            except (OSError, subprocess.SubprocessError):
                pass
            try:
                self.process.wait(timeout=6.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.process.pid, signal.SIGTERM)
                    self.process.wait(timeout=5.0)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(self.process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    self.process.wait(timeout=3.0)
        if self.connection is not None:
            self.connection.close()
        if self.log_file is not None:
            self.log_file.close()
        self.snapshot.restore()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--step-size", type=float, default=0.002)
    parser.add_argument("--pitch-deg", type=float, default=4.0)
    parser.add_argument("--cruise-thrust", type=float, default=0.45)
    parser.add_argument("--json", type=Path, required=True)
    parser.add_argument(
        "--param", action="append", default=[], metavar="NAME=VALUE",
        help="temporary PX4 parameter override; restored after the run",
    )
    arguments = parser.parse_args()
    overrides: dict[str, float] = {}
    for assignment in arguments.param:
        if "=" not in assignment:
            parser.error(f"--param expects NAME=VALUE, got {assignment!r}")
        name, value = assignment.split("=", 1)
        if not name or any(character not in "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_" for character in name):
            parser.error(f"invalid PX4 parameter name {name!r}")
        try:
            overrides[name] = float(value)
        except ValueError:
            parser.error(f"invalid numeric value in --param {assignment!r}")

    report = LateralIdentificationRun(
        arguments.step_size,
        overrides,
        arguments.pitch_deg,
        arguments.cruise_thrust,
    ).execute()
    arguments.json.parent.mkdir(parents=True, exist_ok=True)
    arguments.json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report["metrics"], indent=2))


if __name__ == "__main__":
    main()
