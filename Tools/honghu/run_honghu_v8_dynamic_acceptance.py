#!/usr/bin/env python3
"""End-to-end static, ground-roll, takeoff and flight acceptance for Honghu Wing V8.

This launches the production PX4/Gazebo target, uploads a straight eastbound
mission, and measures the vehicle through MAVLink and Gazebo truth diagnostics.
PX4's BSON parameter files and dataman mission store are snapshotted before
launch and restored byte-for-byte after every run so a test cannot leak
temporary parameters or replace a developer's QGC mission.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import signal
import statistics
import subprocess
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from pymavlink import mavutil


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TEST_MAVLINK_PORT = 15550
ROUTE_LAST_WAYPOINT_SEQUENCE = 5
# Local NED route for lateral guidance identification.  After the eastbound
# takeoff and settling leg, every airborne leg is 1200 m long and advances the
# course anticlockwise by 30 degrees.  It has no reversal, stays within the
# 30000 x 30000 m V8 terrain, and gives the 40 m/s aircraft more than four
# 30-degree-bank turn radii on every leg.
ROUTE_POINTS_NED = {
    0: (0.0, 1000.0),
    1: (0.0, 1600.0),
    2: (600.0, 2639.23),
    3: (1639.23, 3239.23),
    4: (2839.23, 3239.23),
    5: (3878.46, 2639.23),
}
PX4_BIN = ROOT / "build/px4_sitl_default/bin"
ROOTFS = ROOT / "build/px4_sitl_default/rootfs"
MODEL = "honghu_wing_150kg_v8_0"
AERO_TOPIC = f"/model/{MODEL}/honghu_v8/aero_state"
PROPULSION_TOPIC = f"/model/{MODEL}/honghu_v8/propulsion_state"
POSE_TOPIC = "/world/honghu_v8/dynamic_pose/info"
SERVO_TOPICS = {
    f"servo_{index}": f"/model/{MODEL}/servo_{index}"
    # Elevator command traces are retained for direct command-to-joint phase
    # checks during rotation; other surfaces remain available in aero_state.
    for index in (2, 3)
}
SITL_STATE_FILES = (
    ROOTFS / "parameters.bson",
    ROOTFS / "parameters_backup.bson",
    ROOTFS / "dataman",
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


def quaternion_roll_deg(quaternion: list[float] | tuple[float, ...]) -> float:
    """Return the aerospace roll angle from a MAVLink w,x,y,z quaternion."""
    if len(quaternion) != 4:
        return float("nan")
    w, x, y, z = (float(value) for value in quaternion)
    return math.degrees(math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    ))


class SitlStateSnapshot:
    def __init__(self) -> None:
        self._snapshot: dict[Path, tuple[bool, bytes]] = {}

    def capture(self) -> None:
        self._snapshot = {
            path: (path.exists(), path.read_bytes() if path.exists() else b"")
            for path in SITL_STATE_FILES
        }

    def restore(self) -> None:
        for path, (existed, payload) in self._snapshot.items():
            if existed:
                path.parent.mkdir(parents=True, exist_ok=True)
                with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as temporary:
                    temporary.write(payload)
                    temporary_path = Path(temporary.name)
                temporary_path.replace(path)
            elif path.exists():
                path.unlink()


class AeroSampler:
    """Continuously consume Gazebo diagnostics and retain a 20 Hz wall-clock trace."""

    def __init__(self) -> None:
        self.stop_event = threading.Event()
        self.samples: list[dict] = []
        self.propulsion_samples: list[dict] = []
        self.pose_samples: list[dict] = []
        self.servo_command_samples: dict[str, list[dict]] = {
            name: [] for name in SERVO_TOPICS
        }
        self.processes: list[subprocess.Popen] = []
        self.threads: list[threading.Thread] = []

    def start(self) -> None:
        sources = [
            (AERO_TOPIC, self.samples, self._decode_aero),
            (PROPULSION_TOPIC, self.propulsion_samples, self._decode_propulsion),
            (POSE_TOPIC, self.pose_samples, self._decode_pose),
        ]
        sources.extend(
            (topic, self.servo_command_samples[name], self._decode_servo_command)
            for name, topic in SERVO_TOPICS.items()
        )
        for topic, destination, decoder in sources:
            process = subprocess.Popen(
                ["gz", "topic", "-e", "--json-output", "-t", topic],
                cwd=ROOT,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=1,
            )
            self.processes.append(process)
            thread = threading.Thread(
                target=self._consume, args=(process, destination, decoder), daemon=True
            )
            self.threads.append(thread)
            thread.start()

    def close(self) -> None:
        self.stop_event.set()
        for process in self.processes:
            if process.poll() is None:
                process.terminate()
        for process in self.processes:
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        for thread in self.threads:
            thread.join(timeout=2.0)

    def _consume(self, process: subprocess.Popen, destination: list[dict], decoder) -> None:
        last_retained = 0.0
        if process.stdout is None:
            return
        for line in process.stdout:
            if self.stop_event.is_set():
                break
            try:
                payload = json.loads(line.strip())
                frame = payload.get("data", payload)
                now = time.monotonic()
                if now - last_retained < 0.05:
                    continue
                sample = decoder(frame, now)
                if sample is not None:
                    destination.append(sample)
                    last_retained = now
            except (TypeError, ValueError):
                continue

    @staticmethod
    def _decode_aero(frame: list, now: float) -> dict | None:
        if len(frame) != 76:
            return None
        return {
            "wall_time_s": now,
            "airspeed_m_s": float(frame[0]),
            "alpha_deg": float(frame[1]),
            "beta_deg": float(frame[2]),
            "rho_kg_m3": float(frame[3]),
            "alpha_dot_rad_s": float(frame[4]),
            "beta_dot_rad_s": float(frame[5]),
            "body_rates_frd_rad_s": [float(value) for value in frame[6:9]],
            "coefficients": [float(value) for value in frame[9:15]],
            "theta_joint_deg": [float(value) for value in frame[15:23]],
            "delta_doc_deg": [float(value) for value in frame[23:27]],
            "flags": int(round(frame[75])),
        }

    @staticmethod
    def _decode_propulsion(frame: list, now: float) -> dict | None:
        if len(frame) < 9:
            return None
        return {
            "wall_time_s": now,
            "target_throttle": float(frame[0]),
            "filtered_throttle": float(frame[1]),
            "altitude_m": float(frame[2]),
            "airspeed_m_s": float(frame[3]),
            "rpm": float(frame[4]),
            "thrust_n": float(frame[5]),
            "torque_nm": float(frame[6]),
            "fuel_rate": float(frame[7]),
            "flags": int(round(frame[8])),
        }

    @staticmethod
    def _decode_pose(frame: dict, now: float) -> dict | None:
        if not isinstance(frame, dict):
            return None
        model_pose = next(
            (pose for pose in frame.get("pose", []) if pose.get("name") == MODEL),
            None,
        )
        if model_pose is None:
            return None
        position = model_pose.get("position", {})
        orientation = model_pose.get("orientation", {})
        x = float(orientation.get("x", 0.0))
        y = float(orientation.get("y", 0.0))
        z = float(orientation.get("z", 0.0))
        w = float(orientation.get("w", 1.0))
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch_argument = 2.0 * (w * y - z * x)
        pitch = (
            math.copysign(math.pi / 2.0, pitch_argument)
            if abs(pitch_argument) >= 1.0
            else math.asin(pitch_argument)
        )
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return {
            "wall_time_s": now,
            "x_m": float(position.get("x", 0.0)),
            "y_m": float(position.get("y", 0.0)),
            "z_m": float(position.get("z", 0.0)),
            "roll_deg": math.degrees(roll),
            "pitch_deg": math.degrees(pitch),
            "yaw_deg": math.degrees(yaw),
        }

    @staticmethod
    def _decode_servo_command(value, now: float) -> dict | None:
        if not isinstance(value, (int, float)):
            return None
        return {"wall_time_s": now, "angle_rad": float(value), "angle_deg": math.degrees(float(value))}


@dataclass
class TelemetryState:
    local: object | None = None
    attitude: object | None = None
    vfr: object | None = None
    extended: object | None = None
    heartbeat: object | None = None
    attitude_target: object | None = None
    mission_seq: int = -1


class DynamicRun:
    def __init__(
        self,
        scenario: str,
        step_size: float,
        timeout_s: float,
        rwto_pitch_ff: float | None = None,
        parameter_overrides: dict[str, float] | None = None,
        cruise_observation_s: float = 30.0,
        mavlink_port: int = DEFAULT_TEST_MAVLINK_PORT,
        coincident_takeoff: bool = False,
    ) -> None:
        self.scenario = scenario
        self.step_size = step_size
        self.timeout_s = timeout_s
        self.rwto_pitch_ff = rwto_pitch_ff
        self.parameter_overrides = parameter_overrides or {}
        self.cruise_observation_s = cruise_observation_s
        self.mavlink_port = mavlink_port
        self.coincident_takeoff = coincident_takeoff
        self.snapshot = SitlStateSnapshot()
        self.make_process: subprocess.Popen | None = None
        self.make_log_path: Path | None = None
        self.make_log_file = None
        self.connection = None
        self.aero = AeroSampler()
        self.target_system = 1
        self.target_component = 1

    def _assert_clean_runtime(self) -> None:
        result = subprocess.run(
            ["pgrep", "-af", "px4|gz sim|gzserver|ruby.*gz"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        active = [line for line in result.stdout.splitlines() if "pgrep -af" not in line]
        if active:
            raise RuntimeError("existing PX4/Gazebo process detected; refusing to disturb it:\n" + "\n".join(active))

    def start(self) -> None:
        self._assert_clean_runtime()
        self.snapshot.capture()
        output_dir = ROOT / "analysis_outputs"
        output_dir.mkdir(parents=True, exist_ok=True)
        step_label = f"{self.step_size * 1000:g}".replace(".", "p")
        self.make_log_path = output_dir / f"honghu_v8_{self.scenario}_{step_label}ms.log"
        self.make_log_file = self.make_log_path.open("w", encoding="utf-8")
        env = os.environ.copy()
        env["HEADLESS"] = "1"
        env["PX4_GZ_MAX_STEP_SIZE"] = f"{self.step_size:.7f}"
        # Production 4028 aligns the aircraft with the geographic XY mission.
        # Acceptance missions are deliberately eastbound, so override only the
        # test spawn yaw while retaining the validated 0.5145 m spawn height.
        env["PX4_GZ_MODEL_POSE"] = "0,0,0.5145,0,0,0"
        # Keep automated acceptance independent of a concurrently open QGC,
        # which can otherwise claim both the standard remote endpoint and the
        # PX4 UDP partner learned through local port 18570.
        env["PX4_GCS_LOCAL_PORT"] = str(self.mavlink_port + 3000)
        env["PX4_GCS_REMOTE_PORT"] = str(self.mavlink_port)
        env["PX4_GCS_MINIMAL_STREAMS"] = "1"
        self.make_process = subprocess.Popen(
            ["make", "px4_sitl", "gz_honghu_wing_150kg_v8"],
            cwd=ROOT,
            env=env,
            stdout=self.make_log_file,
            stderr=subprocess.STDOUT,
            # Keep an idle pipe open: PX4 pxh redraws its prompt in a tight
            # loop on EOF, which otherwise creates hundreds of MB of log noise.
            stdin=subprocess.PIPE,
            start_new_session=True,
        )
        self.connection = mavutil.mavlink_connection(
            f"udpin:127.0.0.1:{self.mavlink_port}", source_system=250,
        )
        heartbeat = self.connection.wait_heartbeat(timeout=45)
        if heartbeat is None:
            raise TimeoutError(f"PX4 heartbeat not received; see {self.make_log_path}")
        self.target_system = heartbeat.get_srcSystem()
        self.target_component = heartbeat.get_srcComponent()
        self._configure_streams()
        self._set_test_parameters()
        self.aero.start()

    def _configure_streams(self) -> None:
        for message_id, rate_hz in (
            (30, 20), (32, 20), (33, 5), (42, 5), (74, 10), (83, 20), (245, 5),
        ):
            self.connection.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                message_id,
                1_000_000.0 / rate_hz,
                0, 0, 0, 0, 0,
            )

    def _set_test_parameters(self) -> None:
        values = {
            "COM_RC_IN_MODE": 4,
            "COM_DISARM_LAND": -1,
            "COM_DISARM_PRFLT": -1,
            "RWTO_TAXI_TEST": 1 if self.scenario == "taxi" else 0,
        }
        if self.rwto_pitch_ff is not None:
            values["FW_PR_FF_RWTO"] = self.rwto_pitch_ff
        values.update(self.parameter_overrides)
        for name, value in values.items():
            output = run([str(PX4_BIN / "px4-param"), "set", name, str(value)])
            if "ERROR" in output.upper():
                raise RuntimeError(f"failed to set {name}: {output}")

    def _send_gcs_heartbeat(self) -> None:
        self.connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def _drain_mavlink_input(self, limit: int = 20_000) -> int:
        """Discard already queued telemetry while retaining future messages."""
        drained = 0
        while drained < limit:
            message = self.connection.recv_match(blocking=False)
            if message is None:
                break
            drained += 1
        return drained

    def wait_for_position(self) -> tuple[object, object]:
        deadline = time.monotonic() + 35.0
        global_position = None
        local_position = None
        last_heartbeat = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() - last_heartbeat > 0.8:
                self._send_gcs_heartbeat()
                last_heartbeat = time.monotonic()
            message = self.connection.recv_match(
                type=["GLOBAL_POSITION_INT", "LOCAL_POSITION_NED"], blocking=True, timeout=0.5,
            )
            if message is not None and message.get_type() == "GLOBAL_POSITION_INT":
                global_position = message
            elif message is not None:
                local_position = message
            if global_position is not None and local_position is not None:
                return global_position, local_position
        raise TimeoutError("valid global/local position was not received")

    def upload_eastbound_mission(self, global_position: object) -> list[dict]:
        latitude = global_position.lat / 1e7
        longitude = global_position.lon / 1e7
        metres_per_degree_lon = 111_111.0 * math.cos(math.radians(latitude))
        if self.scenario == "taxi":
            specifications = [
                (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0.0, 300.0, 0.0, 12.0),
            ]
        elif self.scenario == "route":
            specifications = [
                # Preserve the takeoff length already proven by earlier V8
                # route runs; the long 30-degree identification legs start
                # only after the following eastbound settling waypoint.
                # PX4 fixed-wing takeoff tracks clearance_altitude + its
                # 10 m kClearanceAltitudeBuffer.  A 40 m clearance therefore
                # hands over continuously to the following 50 m route instead
                # of commanding a 60 -> 50 m descent at the first turn.
                (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0.0, 1000.0, 40.0, 50.0),
                (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0.0, 1600.0, 50.0, 100.0),
                (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 600.0, 2639.23, 50.0, 100.0),
                (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 1639.23, 3239.23, 50.0, 100.0),
                (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2839.23, 3239.23, 50.0, 100.0),
                (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 3878.46, 2639.23, 50.0, 100.0),
                # Keep a landing endpoint for PX4 mission feasibility. The
                # route test stops immediately when this item becomes current.
                (mavutil.mavlink.MAV_CMD_NAV_LAND, 4678.46, 1839.23, 0.0, 100.0),
            ]
        else:
            specifications = [
                # Allow the 150 kg model to complete a shallow straight climb
                # before waypoint capture; a short 800 m takeoff item forces a
                # low-altitude orbit when the altitude criterion is not yet met.
                (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0.0,
                 0.0 if self.coincident_takeoff else 2500.0,
                 40.0 if self.scenario == "flight" else 45.0, 50.0),
                # Keep the flight acceptance altitude continuous when it
                # switches to AUTO_LOITER at 50 m. The takeoff-only scenario
                # retains a 60 m waypoint so it can verify canard retention
                # beyond the 47 m clearance criterion.
                (
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    0.0,
                    8000.0 if self.scenario == "flight" else 3200.0,
                    50.0 if self.scenario == "flight" else 60.0,
                    80.0,
                ),
            ]
            # The flight scenario deliberately uses this same known-valid
            # takeoff mission, then switches to AUTO_LOITER above 50 m. This
            # exercises sustained turning flight without coupling the result to
            # landing-pattern validation or the finite runway terrain extent.
            # Fixed-wing mission feasibility requires a landing endpoint. The
            # acceptance run terminates before this item is reached.
            specifications.append(
                (mavutil.mavlink.MAV_CMD_NAV_LAND, 0.0,
                 10000.0 if self.scenario == "flight" else 5000.0, 0.0, 0.0)
            )
        items = []
        for command, north_m, east_m, altitude_m, acceptance_m in specifications:
            items.append(
                {
                    "command": command,
                    "lat": latitude + north_m / 111_111.0,
                    "lon": longitude + east_m / metres_per_degree_lon,
                    "alt": altitude_m,
                    "acceptance": acceptance_m,
                    "north_m": north_m,
                    "east_m": east_m,
                }
            )

        self.connection.mav.mission_clear_all_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
        clear_deadline = time.monotonic() + 3.0
        while time.monotonic() < clear_deadline:
            clear_ack = self.connection.recv_match(type="MISSION_ACK", blocking=True, timeout=0.3)
            if clear_ack is not None:
                if clear_ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    raise RuntimeError(f"mission clear rejected with ACK type {clear_ack.type}")
                break
        self.connection.mav.mission_count_send(
            self.target_system, self.target_component, len(items),
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
        sent: set[int] = set()
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            self._send_gcs_heartbeat()
            message = self.connection.recv_match(
                type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                blocking=True,
                timeout=1.0,
            )
            if message is None:
                continue
            if message.get_type() == "MISSION_ACK":
                if message.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    raise RuntimeError(f"mission rejected with ACK type {message.type}")
                if len(sent) != len(items):
                    # A delayed clear-all ACK is legal; wait for item requests.
                    self.connection.mav.mission_count_send(
                        self.target_system, self.target_component, len(items),
                        mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                    )
                    continue
                return items
            sequence = int(message.seq)
            if not 0 <= sequence < len(items):
                raise RuntimeError(f"invalid mission request sequence {sequence}")
            item = items[sequence]
            self.connection.mav.mission_item_int_send(
                self.target_system,
                self.target_component,
                sequence,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                item["command"],
                1 if sequence == 0 else 0,
                1,
                0.0,
                float(item["acceptance"]),
                0.0,
                float("nan"),
                int(round(item["lat"] * 1e7)),
                int(round(item["lon"] * 1e7)),
                float(item["alt"]),
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
            sent.add(sequence)
        raise TimeoutError(f"mission upload timed out after sending {sorted(sent)}")

    def arm_and_start_mission(self) -> None:
        run([str(PX4_BIN / "px4-commander"), "mode", "auto:mission"])
        time.sleep(0.5)
        # Mission upload and readiness waits intentionally consume only their
        # response types.  The normal SITL MAVLink profile also sends several
        # 50 Hz streams, so discard that pre-flight history before the timed
        # run.  Otherwise the first tens of seconds describe stale aircraft
        # states while Gazebo diagnostics are live.
        self._drain_mavlink_input()
        # px4-commander writes its success message to the PX4 console rather
        # than this client stdout; its process return code is authoritative.
        run([str(PX4_BIN / "px4-commander"), "arm", "-f"])

    def collect(self, initial_local: object) -> dict:
        state = TelemetryState(local=initial_local)
        samples: list[dict] = []
        start = time.monotonic()
        last_gcs_heartbeat = 0.0
        rotation = None
        liftoff = None
        liftoff_candidate = None
        loiter_start = None
        loiter_ready_since = None
        initial_pose = None
        initial_x = float(initial_local.x)
        initial_y = float(initial_local.y)
        initial_z = float(initial_local.z)

        while time.monotonic() - start < self.timeout_s:
            now = time.monotonic()
            if now - last_gcs_heartbeat > 0.8:
                self._send_gcs_heartbeat()
                last_gcs_heartbeat = now
            # The requested MAVLink streams total about 85 messages/s.  Reading
            # only one message per 20 Hz reporting iteration lets the socket
            # queue fall several seconds behind Gazebo truth, which corrupts
            # event speeds and command-to-joint phase comparisons.  Block for
            # the first message, then drain every message already queued before
            # taking the next 20 Hz snapshot.
            message = self.connection.recv_match(blocking=True, timeout=0.05)
            for _ in range(512):
                if message is None:
                    break
                message_type = message.get_type()
                if message_type == "LOCAL_POSITION_NED":
                    state.local = message
                elif message_type == "ATTITUDE":
                    state.attitude = message
                elif message_type == "VFR_HUD":
                    state.vfr = message
                elif message_type == "EXTENDED_SYS_STATE":
                    state.extended = message
                elif message_type == "HEARTBEAT":
                    state.heartbeat = message
                elif message_type == "ATTITUDE_TARGET":
                    state.attitude_target = message
                elif message_type == "MISSION_CURRENT":
                    state.mission_seq = int(message.seq)
                message = self.connection.recv_match(blocking=False)

            if state.local is None or state.attitude is None or state.vfr is None:
                continue
            if samples and now - samples[-1]["wall_time_s"] < 0.05:
                continue
            along = float(state.local.y) - initial_y
            cross = float(state.local.x) - initial_x
            altitude_gain = initial_z - float(state.local.z)
            latest_pose = self.aero.pose_samples[-1] if self.aero.pose_samples else None
            if initial_pose is None and latest_pose is not None:
                initial_pose = dict(latest_pose)
            pose_is_fresh = latest_pose is not None and now - latest_pose["wall_time_s"] <= 0.5
            gazebo_along = (
                latest_pose["x_m"] - initial_pose["x_m"]
                if pose_is_fresh and initial_pose is not None else float("nan")
            )
            gazebo_cross = (
                latest_pose["y_m"] - initial_pose["y_m"]
                if pose_is_fresh and initial_pose is not None else float("nan")
            )
            gazebo_altitude_gain = (
                latest_pose["z_m"] - initial_pose["z_m"]
                if pose_is_fresh and initial_pose is not None else float("nan")
            )
            sample = {
                "wall_time_s": now,
                "elapsed_s": now - start,
                "along_track_m": along,
                "cross_track_m": cross,
                "altitude_gain_m": altitude_gain,
                "gazebo_along_track_m": gazebo_along,
                "gazebo_cross_track_m": gazebo_cross,
                "gazebo_altitude_gain_m": gazebo_altitude_gain,
                "vx_m_s": float(state.local.vx),
                "vy_m_s": float(state.local.vy),
                "vz_m_s": float(state.local.vz),
                "groundspeed_m_s": math.hypot(float(state.local.vx), float(state.local.vy)),
                "airspeed_m_s": float(state.vfr.airspeed),
                "roll_deg": math.degrees(float(state.attitude.roll)),
                "roll_setpoint_deg": quaternion_roll_deg(state.attitude_target.q)
                if state.attitude_target is not None else float("nan"),
                "roll_rate_deg_s": math.degrees(float(state.attitude.rollspeed)),
                "roll_rate_setpoint_deg_s": math.degrees(float(state.attitude_target.body_roll_rate))
                if state.attitude_target is not None else float("nan"),
                "pitch_deg": math.degrees(float(state.attitude.pitch)),
                "yaw_deg": math.degrees(float(state.attitude.yaw)),
                "throttle_percent": float(state.vfr.throttle),
                "landed_state": int(state.extended.landed_state) if state.extended else None,
                "armed": bool(state.heartbeat and state.heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                "mission_seq": state.mission_seq,
            }
            samples.append(sample)

            if self.scenario in ("takeoff", "flight", "route"):
                if (
                    rotation is None
                    and sample["pitch_deg"] > 2.0
                    and sample["groundspeed_m_s"] > 20.0
                ):
                    rotation = dict(sample)
                if liftoff is None and math.isfinite(sample["gazebo_altitude_gain_m"]):
                    # Record the first plausible wheel-clear event at 0.5 m,
                    # but only confirm it after the aircraft reaches 2 m. This
                    # rejects a transient nose-wheel lift without delaying the
                    # reported liftoff speed to the later clearance crossing.
                    if liftoff_candidate is None and sample["gazebo_altitude_gain_m"] > 0.5 \
                            and sample["groundspeed_m_s"] > 20.0:
                        liftoff_candidate = dict(sample)
                    elif liftoff_candidate is not None and sample["gazebo_altitude_gain_m"] < 0.25:
                        liftoff_candidate = None

                    if liftoff_candidate is not None and sample["gazebo_altitude_gain_m"] > 2.0:
                        liftoff = liftoff_candidate
                # Continue beyond the 45 m clearance altitude and verify
                # the original V3 behavior: both canards retain their fixed
                # takeoff/cruise deflection after the runway state reaches FLY.
                if self.scenario == "takeoff" \
                        and math.isfinite(sample["gazebo_altitude_gain_m"]) \
                        and sample["gazebo_altitude_gain_m"] >= 47.0:
                    recent_aero = [
                        aero for aero in self.aero.samples
                        if now - aero["wall_time_s"] <= 0.75
                    ]
                    if len(recent_aero) >= 3 and all(
                        3.5 <= aero["delta_doc_deg"][3] <= 4.5
                        and 3.5 <= aero["theta_joint_deg"][6] <= 4.5
                        and 3.5 <= aero["theta_joint_deg"][7] <= 4.5
                        for aero in recent_aero
                    ):
                        break
                elif self.scenario == "flight" and loiter_start is None:
                    # Do not force a loiter transition while the takeoff climb
                    # still carries several m/s of vertical speed. Require a
                    # short, genuinely level handoff near the 50 m target so
                    # the test measures normal turning flight rather than a
                    # climb-to-hold step transient.
                    loiter_ready = (
                        math.isfinite(sample["gazebo_altitude_gain_m"])
                        and 47.0 <= sample["gazebo_altitude_gain_m"] <= 55.0
                        and abs(sample["vz_m_s"]) <= 1.0
                    )
                    if loiter_ready:
                        if loiter_ready_since is None:
                            loiter_ready_since = now
                        elif now - loiter_ready_since >= 0.75:
                            run([str(PX4_BIN / "px4-commander"), "mode", "auto:loiter"])
                            loiter_start = now
                    else:
                        loiter_ready_since = None
                elif (
                    self.scenario == "flight"
                    and loiter_start is not None
                    and now - loiter_start >= self.cruise_observation_s
                ):
                    break
                elif (
                    self.scenario == "route"
                    and state.mission_seq > ROUTE_LAST_WAYPOINT_SEQUENCE
                ):
                    break
            elif self.scenario == "taxi" and along >= 210.0:
                break

        if not samples:
            raise RuntimeError("no complete telemetry samples collected")
        return self._evaluate(samples, rotation, liftoff, loiter_start)

    def _evaluate(
        self,
        samples: list[dict],
        rotation: dict | None,
        liftoff: dict | None,
        loiter_start: float | None,
    ) -> dict:
        if self.scenario == "static":
            px4_horizontal_drift = max(
                math.hypot(sample["along_track_m"], sample["cross_track_m"]) for sample in samples
            )
            px4_altitude_values = [sample["altitude_gain_m"] for sample in samples]
            pose_samples = [
                pose for pose in self.aero.pose_samples
                if pose["wall_time_s"] >= samples[0]["wall_time_s"]
            ]
            if pose_samples:
                pose_origin = pose_samples[0]
                gazebo_horizontal_drift = max(
                    math.hypot(pose["x_m"] - pose_origin["x_m"], pose["y_m"] - pose_origin["y_m"])
                    for pose in pose_samples
                )
                gazebo_altitude_values = [pose["z_m"] for pose in pose_samples]
                gazebo_altitude_range = max(gazebo_altitude_values) - min(gazebo_altitude_values)
                gazebo_roll_max = max(abs(pose["roll_deg"]) for pose in pose_samples)
                gazebo_pitch_max = max(abs(pose["pitch_deg"]) for pose in pose_samples)
            else:
                gazebo_horizontal_drift = float("nan")
                gazebo_altitude_range = float("nan")
                gazebo_roll_max = float("nan")
                gazebo_pitch_max = float("nan")
            metrics = {
                "elapsed_s": samples[-1]["elapsed_s"],
                "gazebo_pose_sample_count": len(pose_samples),
                "gazebo_horizontal_drift_max_m": gazebo_horizontal_drift,
                "gazebo_altitude_range_m": gazebo_altitude_range,
                "gazebo_roll_max_abs_deg": gazebo_roll_max,
                "gazebo_pitch_max_abs_deg": gazebo_pitch_max,
                "px4_estimator_horizontal_drift_max_m": px4_horizontal_drift,
                "px4_estimator_altitude_range_m": max(px4_altitude_values) - min(px4_altitude_values),
                "px4_estimator_groundspeed_max_m_s": max(sample["groundspeed_m_s"] for sample in samples),
                "px4_estimator_roll_max_abs_deg": max(abs(sample["roll_deg"]) for sample in samples),
                "px4_estimator_pitch_max_abs_deg": max(abs(sample["pitch_deg"]) for sample in samples),
            }
            checks = {
                "observed_at_least_60s": metrics["elapsed_s"] >= 60.0,
                "gazebo_pose_samples_available": len(pose_samples) >= 100,
                "gazebo_horizontal_drift_below_0p05m": math.isfinite(gazebo_horizontal_drift)
                and gazebo_horizontal_drift < 0.05,
                "gazebo_altitude_range_below_0p02m": math.isfinite(gazebo_altitude_range)
                and gazebo_altitude_range < 0.02,
                "gazebo_attitude_below_0p3deg": math.isfinite(gazebo_roll_max)
                and math.isfinite(gazebo_pitch_max)
                and gazebo_roll_max < 0.3
                and gazebo_pitch_max < 0.3,
            }
        elif self.scenario == "taxi":
            steady = [sample for sample in samples if sample["along_track_m"] >= 20.0]
            if not steady:
                steady = samples
            rms_cross = math.sqrt(statistics.fmean(sample["cross_track_m"] ** 2 for sample in steady))
            mean_speed = statistics.fmean(sample["groundspeed_m_s"] for sample in steady)
            metrics = {
                "distance_m": max(sample["along_track_m"] for sample in samples),
                "cross_track_rms_m": rms_cross,
                "cross_track_max_abs_m": max(abs(sample["cross_track_m"]) for sample in steady),
                "steady_groundspeed_mean_m_s": mean_speed,
                "steady_groundspeed_max_m_s": max(sample["groundspeed_m_s"] for sample in steady),
                "altitude_gain_max_m": max(sample["altitude_gain_m"] for sample in samples),
                "roll_max_abs_deg": max(abs(sample["roll_deg"]) for sample in samples),
                "pitch_max_abs_deg": max(abs(sample["pitch_deg"]) for sample in samples),
            }
            checks = {
                "distance_at_least_200m": metrics["distance_m"] >= 200.0,
                "cross_track_rms_below_0p5m": rms_cross < 0.5,
                "mean_speed_7_to_9m_s": 7.0 <= mean_speed <= 9.0,
                "remained_on_ground": metrics["altitude_gain_max_m"] < 0.5,
            }
        elif self.scenario == "takeoff":
            takeoff_pose_samples = [
                pose for pose in self.aero.pose_samples
                if samples[0]["wall_time_s"] <= pose["wall_time_s"] <= samples[-1]["wall_time_s"]
            ]
            pose_origin = takeoff_pose_samples[0] if takeoff_pose_samples else None
            liftoff_time = liftoff["wall_time_s"] if liftoff is not None else samples[-1]["wall_time_s"]
            ground_pose_samples = [
                pose for pose in takeoff_pose_samples
                if pose["wall_time_s"] <= liftoff_time
            ]
            takeoff_truth_pitch_max = max(
                (abs(pose["pitch_deg"]) for pose in takeoff_pose_samples),
                default=float("nan"),
            )
            ground_truth_cross_track_max = max(
                (abs(pose["y_m"] - pose_origin["y_m"]) for pose in ground_pose_samples),
                default=float("nan"),
            ) if pose_origin is not None else float("nan")
            canard_samples = [
                sample for sample in self.aero.samples
                if len(sample["theta_joint_deg"]) >= 8 and len(sample["delta_doc_deg"]) >= 4
            ]
            canard_takeoff_peak_deg = max(
                (sample["delta_doc_deg"][3] for sample in canard_samples), default=float("nan")
            )
            cruise_start_time = next(
                (sample["wall_time_s"] for sample in samples if sample["altitude_gain_m"] >= 46.0),
                None,
            )
            cruise_canard_samples = [
                sample for sample in canard_samples
                if cruise_start_time is not None and sample["wall_time_s"] >= cruise_start_time
            ]
            cruise_canard_min_deg = min(
                (
                    min(
                        sample["delta_doc_deg"][3],
                        sample["theta_joint_deg"][6],
                        sample["theta_joint_deg"][7],
                    )
                    for sample in cruise_canard_samples[-10:]
                ),
                default=float("nan"),
            )
            cruise_canard_max_deg = max(
                (
                    max(
                        sample["delta_doc_deg"][3],
                        sample["theta_joint_deg"][6],
                        sample["theta_joint_deg"][7],
                    )
                    for sample in cruise_canard_samples[-10:]
                ),
                default=float("nan"),
            )
            canard_pair_max_error_deg = max(
                (
                    abs(sample["theta_joint_deg"][6] - sample["theta_joint_deg"][7])
                    for sample in canard_samples
                    if sample["delta_doc_deg"][3] > 3.5
                ),
                default=float("nan"),
            )
            metrics = {
                "rotation": rotation,
                "liftoff": liftoff,
                "altitude_gain_final_m": samples[-1]["altitude_gain_m"],
                "gazebo_altitude_final_m": samples[-1]["gazebo_altitude_gain_m"],
                "gazebo_ground_cross_track_max_abs_to_liftoff_m": ground_truth_cross_track_max,
                "takeoff_truth_pitch_max_abs_deg": takeoff_truth_pitch_max,
                "ground_roll_distance_m": liftoff["along_track_m"] if liftoff else None,
                "airspeed_max_m_s": max(sample["airspeed_m_s"] for sample in samples),
                "canard_takeoff_peak_deg": canard_takeoff_peak_deg,
                "canard_pair_max_error_deg": canard_pair_max_error_deg,
                "canard_cruise_min_deg": cruise_canard_min_deg,
                "canard_cruise_max_deg": cruise_canard_max_deg,
            }
            checks = {
                "rotation_detected": rotation is not None,
                "rotation_at_or_below_45m_s": rotation is not None and rotation["airspeed_m_s"] <= 45.0,
                "liftoff_detected": liftoff is not None,
                "liftoff_before_45m_s": liftoff is not None and liftoff["airspeed_m_s"] <= 45.0,
                "ground_cross_track_below_3m": math.isfinite(ground_truth_cross_track_max)
                and ground_truth_cross_track_max < 3.0,
                "takeoff_truth_pitch_below_12deg": math.isfinite(takeoff_truth_pitch_max)
                and takeoff_truth_pitch_max <= 12.0,
                "climbed_10m": metrics["gazebo_altitude_final_m"] >= 10.0,
                "canards_deployed_at_takeoff": math.isfinite(canard_takeoff_peak_deg)
                and 3.5 <= canard_takeoff_peak_deg <= 4.5,
                "canard_pair_synchronized": math.isfinite(canard_pair_max_error_deg)
                and canard_pair_max_error_deg < 0.25,
                "canards_hold_v3_cruise_deflection": math.isfinite(cruise_canard_min_deg)
                and math.isfinite(cruise_canard_max_deg)
                and cruise_canard_min_deg >= 3.5
                and cruise_canard_max_deg <= 4.5,
            }
        elif self.scenario == "route":
            ground_samples = [
                sample for sample in samples
                if liftoff is not None and sample["wall_time_s"] <= liftoff["wall_time_s"]
            ]
            route_start_time = next(
                (
                    sample["wall_time_s"] for sample in samples
                    if 1 <= sample["mission_seq"] <= ROUTE_LAST_WAYPOINT_SEQUENCE
                    and sample["altitude_gain_m"] >= 35.0
                ),
                None,
            )
            route_samples = [
                sample for sample in samples
                if route_start_time is not None
                and sample["wall_time_s"] >= route_start_time
                and 1 <= sample["mission_seq"] <= ROUTE_LAST_WAYPOINT_SEQUENCE
            ]
            track_errors: list[float] = []
            segment_errors: dict[str, list[float]] = {}
            stable_track_errors: list[float] = []
            stable_segment_errors: dict[str, list[float]] = {}
            for sample in route_samples:
                sequence = sample["mission_seq"]
                start_n, start_e = ROUTE_POINTS_NED[sequence - 1]
                end_n, end_e = ROUTE_POINTS_NED[sequence]
                delta_n = end_n - start_n
                delta_e = end_e - start_e
                length = math.hypot(delta_n, delta_e)
                signed_error = (
                    delta_n * (sample["along_track_m"] - start_e)
                    - delta_e * (sample["cross_track_m"] - start_n)
                ) / length
                progress = (
                    (sample["cross_track_m"] - start_n) * delta_n
                    + (sample["along_track_m"] - start_e) * delta_e
                ) / (length * length)
                track_errors.append(signed_error)
                segment_errors.setdefault(str(sequence), []).append(signed_error)
                # Exclude the commanded corner and the next fly-by transition
                # when measuring settled straight-line tracking.
                if 0.35 <= progress <= 0.80:
                    stable_track_errors.append(signed_error)
                    stable_segment_errors.setdefault(str(sequence), []).append(signed_error)

            route_altitudes = [sample["altitude_gain_m"] for sample in route_samples]
            route_airspeeds = [sample["airspeed_m_s"] for sample in route_samples]
            canard_values = [
                value
                for aero in self.aero.samples
                if route_samples
                and route_samples[0]["wall_time_s"] <= aero["wall_time_s"] <= route_samples[-1]["wall_time_s"]
                for value in (
                    aero["delta_doc_deg"][3],
                    aero["theta_joint_deg"][6],
                    aero["theta_joint_deg"][7],
                )
            ]
            sorted_abs_errors = sorted(abs(error) for error in track_errors)
            p95_index = max(0, math.ceil(0.95 * len(sorted_abs_errors)) - 1)
            track_rms = math.sqrt(statistics.fmean(error * error for error in track_errors)) \
                if track_errors else float("nan")
            track_p95 = sorted_abs_errors[p95_index] if sorted_abs_errors else float("nan")
            sorted_stable_abs_errors = sorted(abs(error) for error in stable_track_errors)
            stable_p95_index = max(0, math.ceil(0.95 * len(sorted_stable_abs_errors)) - 1)
            stable_track_rms = math.sqrt(statistics.fmean(
                error * error for error in stable_track_errors
            )) if stable_track_errors else float("nan")
            stable_track_p95 = (
                sorted_stable_abs_errors[stable_p95_index]
                if sorted_stable_abs_errors else float("nan")
            )
            segment_metrics = {
                sequence: {
                    "sample_count": len(errors),
                    "rms_m": math.sqrt(statistics.fmean(error * error for error in errors)),
                    "max_abs_m": max(abs(error) for error in errors),
                    "stable_sample_count": len(stable_segment_errors.get(sequence, [])),
                    "stable_rms_m": math.sqrt(statistics.fmean(
                        error * error for error in stable_segment_errors[sequence]
                    )) if stable_segment_errors.get(sequence) else float("nan"),
                    "stable_max_abs_m": max(
                        (abs(error) for error in stable_segment_errors.get(sequence, [])),
                        default=float("nan"),
                    ),
                }
                for sequence, errors in segment_errors.items()
            }
            roll_errors = [
                sample["roll_setpoint_deg"] - sample["roll_deg"]
                for sample in route_samples
                if math.isfinite(sample["roll_setpoint_deg"])
            ]
            roll_rate_errors = [
                sample["roll_rate_setpoint_deg_s"] - sample["roll_rate_deg_s"]
                for sample in route_samples
                if math.isfinite(sample["roll_rate_setpoint_deg_s"])
            ]
            metrics = {
                "rotation": rotation,
                "liftoff": liftoff,
                "mission_sequence_max": max(sample["mission_seq"] for sample in samples),
                "route_sample_count": len(route_samples),
                "route_track_error_rms_m": track_rms,
                "route_track_error_p95_abs_m": track_p95,
                "route_track_error_max_abs_m": max(sorted_abs_errors, default=float("nan")),
                "route_stable_track_sample_count": len(stable_track_errors),
                "route_stable_track_error_rms_m": stable_track_rms,
                "route_stable_track_error_p95_abs_m": stable_track_p95,
                "route_segment_metrics": segment_metrics,
                "route_altitude_min_m": min(route_altitudes, default=float("nan")),
                "route_altitude_max_m": max(route_altitudes, default=float("nan")),
                "route_airspeed_min_m_s": min(route_airspeeds, default=float("nan")),
                "route_airspeed_max_m_s": max(route_airspeeds, default=float("nan")),
                "route_roll_max_abs_deg": max((abs(sample["roll_deg"]) for sample in route_samples), default=float("nan")),
                "route_roll_setpoint_max_abs_deg": max(
                    (abs(sample["roll_setpoint_deg"]) for sample in route_samples
                     if math.isfinite(sample["roll_setpoint_deg"])),
                    default=float("nan"),
                ),
                "route_roll_tracking_error_rms_deg": math.sqrt(statistics.fmean(
                    error * error for error in roll_errors
                )) if roll_errors else float("nan"),
                "route_roll_rate_tracking_error_rms_deg_s": math.sqrt(statistics.fmean(
                    error * error for error in roll_rate_errors
                )) if roll_rate_errors else float("nan"),
                "route_pitch_max_abs_deg": max((abs(sample["pitch_deg"]) for sample in route_samples), default=float("nan")),
                "ground_cross_track_max_abs_m": max(
                    (abs(sample["gazebo_cross_track_m"]) for sample in ground_samples),
                    key=lambda value: value if math.isfinite(value) else -math.inf,
                    default=float("nan"),
                ),
                "takeoff_truth_pitch_max_abs_deg": max(
                    (abs(pose["pitch_deg"]) for pose in self.aero.pose_samples
                     if samples[0]["wall_time_s"] <= pose["wall_time_s"]
                     <= (route_start_time or samples[-1]["wall_time_s"])),
                    default=float("nan"),
                ),
                "route_canard_min_deg": min(canard_values, default=float("nan")),
                "route_canard_max_deg": max(canard_values, default=float("nan")),
            }
            checks = {
                "rotation_detected": rotation is not None,
                "liftoff_detected_from_gazebo_truth": liftoff is not None,
                "liftoff_before_45m_s": liftoff is not None and liftoff["airspeed_m_s"] <= 45.0,
                "ground_cross_track_below_3m": math.isfinite(metrics["ground_cross_track_max_abs_m"])
                and metrics["ground_cross_track_max_abs_m"] < 3.0,
                "takeoff_truth_pitch_below_12deg": math.isfinite(metrics["takeoff_truth_pitch_max_abs_deg"])
                and metrics["takeoff_truth_pitch_max_abs_deg"] <= 12.0,
                "route_waypoints_completed": metrics["mission_sequence_max"] > ROUTE_LAST_WAYPOINT_SEQUENCE,
                "route_track_rms_below_60m": math.isfinite(track_rms) and track_rms < 60.0,
                "route_track_p95_below_120m": math.isfinite(track_p95) and track_p95 < 120.0,
                "route_stable_track_rms_below_30m": math.isfinite(stable_track_rms)
                and stable_track_rms < 30.0,
                "route_stable_track_p95_below_60m": math.isfinite(stable_track_p95)
                and stable_track_p95 < 60.0,
                "route_altitude_35_to_70m": route_altitudes
                and min(route_altitudes) >= 35.0 and max(route_altitudes) <= 70.0,
                "route_airspeed_25_to_60m_s": route_airspeeds
                and min(route_airspeeds) >= 25.0 and max(route_airspeeds) <= 60.0,
                "route_attitude_bounded": math.isfinite(metrics["route_roll_max_abs_deg"])
                and math.isfinite(metrics["route_pitch_max_abs_deg"])
                and metrics["route_roll_max_abs_deg"] < 35.0
                and metrics["route_roll_setpoint_max_abs_deg"] <= 30.5
                and metrics["route_pitch_max_abs_deg"] < 15.0,
                "canards_hold_v3_cruise_deflection": math.isfinite(metrics["route_canard_min_deg"])
                and math.isfinite(metrics["route_canard_max_deg"])
                and metrics["route_canard_min_deg"] >= 3.5
                and metrics["route_canard_max_deg"] <= 4.5,
            }
        else:
            ground_samples = [
                sample for sample in samples
                if liftoff is not None and sample["wall_time_s"] <= liftoff["wall_time_s"]
            ]
            takeoff_end = loiter_start if loiter_start is not None else samples[-1]["wall_time_s"]
            takeoff_pose_samples = [
                pose for pose in self.aero.pose_samples
                if samples[0]["wall_time_s"] <= pose["wall_time_s"] <= takeoff_end
            ]
            takeoff_truth_pitch_max = max(
                (abs(pose["pitch_deg"]) for pose in takeoff_pose_samples),
                default=float("nan"),
            )
            cruise_samples = [
                sample for sample in samples
                if loiter_start is not None
                and sample["wall_time_s"] >= loiter_start + 5.0
                and math.isfinite(sample["gazebo_altitude_gain_m"])
            ]
            if cruise_samples:
                cruise_start = cruise_samples[0]["wall_time_s"]
                cruise_end = cruise_samples[-1]["wall_time_s"]
                cruise_aero = [
                    sample for sample in self.aero.samples
                    if cruise_start <= sample["wall_time_s"] <= cruise_end
                ]
            else:
                cruise_aero = []
            canard_values = [
                value
                for sample in cruise_aero
                if len(sample["theta_joint_deg"]) >= 8 and len(sample["delta_doc_deg"]) >= 4
                for value in (
                    sample["delta_doc_deg"][3],
                    sample["theta_joint_deg"][6],
                    sample["theta_joint_deg"][7],
                )
            ]
            cruise_duration = (
                cruise_samples[-1]["wall_time_s"] - cruise_samples[0]["wall_time_s"]
                if len(cruise_samples) >= 2 else 0.0
            )
            metrics = {
                "rotation": rotation,
                "liftoff": liftoff,
                "loiter_started": loiter_start is not None,
                "loiter_settled_duration_s": cruise_duration,
                "gazebo_distance_at_loiter_end_m": samples[-1]["gazebo_along_track_m"],
                "gazebo_altitude_final_m": samples[-1]["gazebo_altitude_gain_m"],
                "ground_cross_track_max_abs_m": max(
                    (
                        abs(sample["gazebo_cross_track_m"])
                        for sample in ground_samples
                        if math.isfinite(sample["gazebo_cross_track_m"])
                    ),
                    default=float("nan"),
                ),
                "takeoff_truth_pitch_max_abs_deg": takeoff_truth_pitch_max,
                "cruise_sample_count": len(cruise_samples),
                "cruise_altitude_min_m": min(
                    (sample["gazebo_altitude_gain_m"] for sample in cruise_samples),
                    default=float("nan"),
                ),
                "cruise_altitude_max_m": max(
                    (sample["gazebo_altitude_gain_m"] for sample in cruise_samples),
                    default=float("nan"),
                ),
                "cruise_airspeed_min_m_s": min(
                    (sample["airspeed_m_s"] for sample in cruise_samples), default=float("nan")
                ),
                "cruise_airspeed_max_m_s": max(
                    (sample["airspeed_m_s"] for sample in cruise_samples), default=float("nan")
                ),
                "cruise_roll_max_abs_deg": max(
                    (abs(sample["roll_deg"]) for sample in cruise_samples), default=float("nan")
                ),
                "cruise_pitch_max_abs_deg": max(
                    (abs(sample["pitch_deg"]) for sample in cruise_samples), default=float("nan")
                ),
                "cruise_alpha_max_abs_deg": max(
                    (abs(sample["alpha_deg"]) for sample in cruise_aero), default=float("nan")
                ),
                "cruise_beta_max_abs_deg": max(
                    (abs(sample["beta_deg"]) for sample in cruise_aero), default=float("nan")
                ),
                "cruise_canard_min_deg": min(canard_values, default=float("nan")),
                "cruise_canard_max_deg": max(canard_values, default=float("nan")),
            }
            checks = {
                "rotation_detected": rotation is not None,
                "liftoff_detected_from_gazebo_truth": liftoff is not None,
                "liftoff_before_45m_s": liftoff is not None and liftoff["airspeed_m_s"] <= 45.0,
                "ground_cross_track_below_3m": math.isfinite(metrics["ground_cross_track_max_abs_m"])
                and metrics["ground_cross_track_max_abs_m"] < 3.0,
                "takeoff_truth_pitch_below_12deg": math.isfinite(takeoff_truth_pitch_max)
                and takeoff_truth_pitch_max <= 12.0,
                "auto_loiter_entered_above_50m": loiter_start is not None,
                "stable_flight_observed_at_least_24s": cruise_duration >= 24.0,
                # Use the same bounded low-altitude envelope as the route
                # acceptance. A 30 deg loiter naturally trades a few metres
                # of height while TECS establishes load-factor compensation;
                # 35 m remains well clear of the ground without hiding a real
                # altitude-control loss.
                "cruise_altitude_35_to_70m": math.isfinite(metrics["cruise_altitude_min_m"])
                and metrics["cruise_altitude_min_m"] >= 35.0
                and metrics["cruise_altitude_max_m"] <= 70.0,
                "cruise_attitude_bounded": math.isfinite(metrics["cruise_roll_max_abs_deg"])
                and math.isfinite(metrics["cruise_pitch_max_abs_deg"])
                and metrics["cruise_roll_max_abs_deg"] < 35.0
                and metrics["cruise_pitch_max_abs_deg"] < 15.0,
                "cruise_airspeed_25_to_60m_s": math.isfinite(metrics["cruise_airspeed_min_m_s"])
                and metrics["cruise_airspeed_min_m_s"] >= 25.0
                and metrics["cruise_airspeed_max_m_s"] <= 60.0,
                "cruise_angles_in_table_core": math.isfinite(metrics["cruise_alpha_max_abs_deg"])
                and math.isfinite(metrics["cruise_beta_max_abs_deg"])
                and metrics["cruise_alpha_max_abs_deg"] < 12.0
                and metrics["cruise_beta_max_abs_deg"] < 5.0,
                "canards_hold_v3_cruise_deflection": math.isfinite(metrics["cruise_canard_min_deg"])
                and math.isfinite(metrics["cruise_canard_max_deg"])
                and metrics["cruise_canard_min_deg"] >= 3.5
                and metrics["cruise_canard_max_deg"] <= 4.5,
            }
        return {
            "status": "PASS" if all(checks.values()) else "FAIL",
            "scenario": self.scenario,
            "step_size_s": self.step_size,
            "checks": checks,
            "metrics": metrics,
            "sample_count": len(samples),
            "samples": samples,
            "aero_sample_count": len(self.aero.samples),
            "aero_samples": self.aero.samples,
            "propulsion_sample_count": len(self.aero.propulsion_samples),
            "propulsion_samples": self.aero.propulsion_samples,
            "gazebo_pose_sample_count": len(self.aero.pose_samples),
            "gazebo_pose_samples": self.aero.pose_samples,
            "servo_command_sample_counts": {
                name: len(values) for name, values in self.aero.servo_command_samples.items()
            },
            "servo_command_samples": self.aero.servo_command_samples,
        }

    def execute(self) -> dict:
        try:
            self.start()
            global_position, local_position = self.wait_for_position()
            if self.scenario == "static":
                mission = []
            else:
                mission = self.upload_eastbound_mission(global_position)
                self.arm_and_start_mission()
            report = self.collect(local_position)
            report["mission"] = mission
            report["make_log"] = str(self.make_log_path)
            return report
        finally:
            self.stop()

    def stop(self) -> None:
        self.aero.close()
        if self.make_process is not None:
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
                self.make_process.wait(timeout=6.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.make_process.pid, signal.SIGTERM)
                    self.make_process.wait(timeout=5.0)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(self.make_process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    self.make_process.wait(timeout=3.0)
        if self.connection is not None:
            self.connection.close()
        if self.make_log_file is not None:
            self.make_log_file.close()
        self.snapshot.restore()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", choices=("static", "taxi", "takeoff", "flight", "route"))
    parser.add_argument(
        "--step-size", type=float, default=0.002,
        help="Gazebo physics step [s]; V8's validated real-time operating point is 0.002",
    )
    parser.add_argument("--timeout", type=float)
    parser.add_argument(
        "--mavlink-port", type=int, default=DEFAULT_TEST_MAVLINK_PORT,
        help="local UDP telemetry port; select another port when QGC owns 15550",
    )
    parser.add_argument(
        "--cruise-observation", type=float, default=30.0,
        help="flight-mode loiter observation time after the level handoff [s]",
    )
    parser.add_argument("--json", type=Path)
    parser.add_argument(
        "--rwto-pitch-ff", type=float,
        help="temporary FW_PR_FF_RWTO override for bounded takeoff tuning",
    )
    parser.add_argument(
        "--param", action="append", default=[], metavar="NAME=VALUE",
        help="temporary numeric PX4 parameter override; may be repeated",
    )
    parser.add_argument(
        "--coincident-takeoff", action="store_true",
        help="place TAKEOFF at the launch position to test RWTO_DIR_MIN next-waypoint fallback",
    )
    parser.add_argument("--no-assert", action="store_true", help="write measurements without a failing exit code")
    arguments = parser.parse_args()
    if not 1024 <= arguments.mavlink_port <= 62535:
        parser.error("--mavlink-port must be in 1024..62535")
    if arguments.coincident_takeoff and arguments.scenario not in ("takeoff", "flight"):
        parser.error("--coincident-takeoff is supported only by takeoff and flight scenarios")
    default_timeouts = {
        "static": 62.0,
        "taxi": 70.0,
        "takeoff": 110.0,
        "flight": 150.0,
        "route": 190.0,
    }
    timeout_s = arguments.timeout or default_timeouts[arguments.scenario]
    parameter_overrides: dict[str, float] = {}
    for assignment in arguments.param:
        if "=" not in assignment:
            parser.error(f"--param expects NAME=VALUE, got {assignment!r}")
        name, value = assignment.split("=", 1)
        if not name or any(character not in "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_" for character in name):
            parser.error(f"invalid PX4 parameter name {name!r}")
        try:
            parameter_overrides[name] = float(value)
        except ValueError:
            parser.error(f"invalid numeric value in --param {assignment!r}")
    runner = DynamicRun(
        arguments.scenario,
        arguments.step_size,
        timeout_s,
        arguments.rwto_pitch_ff,
        parameter_overrides,
        arguments.cruise_observation,
        arguments.mavlink_port,
        arguments.coincident_takeoff,
    )
    report = runner.execute()
    if arguments.json:
        arguments.json.parent.mkdir(parents=True, exist_ok=True)
        arguments.json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Honghu V8 {arguments.scenario} acceptance: {report['status']}")
    print(json.dumps(report["metrics"], indent=2))
    print(json.dumps(report["checks"], indent=2))
    if report["status"] != "PASS" and not arguments.no_assert:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
