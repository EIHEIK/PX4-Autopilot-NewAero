#!/usr/bin/env python3
"""End-to-end static, ground-roll, takeoff and flight acceptance for Honghu Wing V8.

This launches the production PX4/Gazebo target, uploads a straight eastbound
mission, and measures the vehicle through MAVLink. Gazebo truth diagnostics
are read from ULog only after PX4 and Gazebo have stopped; the acceptance tool
never starts external topic observers during closed-loop flight.
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
import time
from dataclasses import dataclass
from pathlib import Path

from pymavlink import mavutil
import numpy as np
from pyulog import ULog

from upload_qgc_plan import load_items


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_STANDARD_PLAN = Path("/home/fly/px4_reference_docs/current/模仿XY航线规划.plan")
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


class OfflineGazeboDiagnostics:
    """Read Gazebo truth from the completed ULog, never during simulation."""

    def __init__(self) -> None:
        self.samples: list[dict] = []
        self.propulsion_samples: list[dict] = []
        self.pose_samples: list[dict] = []
        self.servo_command_samples: dict[str, list[dict]] = {}
        self.ulog_path: Path | None = None

    @staticmethod
    def _dataset(ulog: ULog, name: str):
        return next((item for item in ulog.data_list if item.name == name), None)

    @staticmethod
    def _sample_time_s(dataset) -> np.ndarray:
        if "timestamp_sample" in dataset.data:
            source = np.asarray(dataset.data["timestamp_sample"], dtype=float)
            if len(source) and np.count_nonzero(source > 0) > len(source) // 2:
                return source * 1e-6
        return np.asarray(dataset.data["timestamp"], dtype=float) * 1e-6

    @staticmethod
    def _attitude_deg(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float]:
        w, x, y, z = quaternion
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch_argument = 2.0 * (w * y - z * x)
        pitch = math.copysign(math.pi / 2.0, pitch_argument) \
            if abs(pitch_argument) >= 1.0 else math.asin(pitch_argument)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return tuple(math.degrees(value) for value in (roll, pitch, yaw))

    @staticmethod
    def _wall_time_mapper(live_samples: list[dict]):
        sim_time = np.asarray([sample["sim_time_s"] for sample in live_samples], dtype=float)
        wall_time = np.asarray([sample["wall_time_s"] for sample in live_samples], dtype=float)

        def convert(source_time: np.ndarray) -> np.ndarray:
            return np.interp(source_time, sim_time, wall_time, left=wall_time[0], right=wall_time[-1])

        return convert

    def load(self, ulog_path: Path, live_samples: list[dict]) -> None:
        """Populate the former live-sampler interface after PX4 has stopped."""
        self.ulog_path = ulog_path
        ulog = ULog(
            str(ulog_path),
            message_name_filter_list=[
                "honghu_v8_aero_state", "honghu_v8_propulsion_state",
                "vehicle_local_position_groundtruth", "vehicle_attitude_groundtruth",
            ],
        )
        to_wall = self._wall_time_mapper(live_samples)
        aero = self._dataset(ulog, "honghu_v8_aero_state")
        if aero is not None:
            source_time = self._sample_time_s(aero)
            wall_time = to_wall(source_time)
            for index, now in enumerate(wall_time):
                self.samples.append(
                    {
                        "wall_time_s": float(now),
                        "sim_time_s": float(source_time[index]),
                        "airspeed_m_s": float(aero.data["airspeed_m_s"][index]),
                        "alpha_deg": float(aero.data["alpha_deg"][index]),
                        "beta_deg": float(aero.data["beta_deg"][index]),
                        "rho_kg_m3": float(aero.data["rho_kg_m3"][index]),
                        "alpha_dot_rad_s": float(aero.data["alpha_dot_rad_s"][index]),
                        "beta_dot_rad_s": float(aero.data["beta_dot_rad_s"][index]),
                        "body_rates_frd_rad_s": [
                            float(aero.data[f"body_rates_frd_rad_s[{axis}]"][index])
                            for axis in range(3)
                        ],
                        "coefficients": [
                            float(aero.data[f"coefficients[{axis}]"][index]) for axis in range(6)
                        ],
                        "theta_joint_deg": [
                            float(aero.data[f"joint_angles_deg[{axis}]"][index]) for axis in range(8)
                        ],
                        "delta_doc_deg": [
                            float(aero.data[f"delta_doc_deg[{axis}]"][index]) for axis in range(4)
                        ],
                        "flags": int(aero.data["flags"][index]),
                    }
                )

        propulsion = self._dataset(ulog, "honghu_v8_propulsion_state")
        if propulsion is not None:
            source_time = self._sample_time_s(propulsion)
            wall_time = to_wall(source_time)
            fields = (
                "target_throttle", "filtered_throttle", "altitude_m", "airspeed_m_s",
                "rpm", "thrust_n", "torque_nm", "fuel_rate",
            )
            for index, now in enumerate(wall_time):
                sample = {
                    "wall_time_s": float(now),
                    "sim_time_s": float(source_time[index]),
                    "flags": int(propulsion.data["flags"][index]),
                }
                sample.update({field: float(propulsion.data[field][index]) for field in fields})
                self.propulsion_samples.append(sample)

        local_position = self._dataset(ulog, "vehicle_local_position_groundtruth")
        attitude = self._dataset(ulog, "vehicle_attitude_groundtruth")
        if local_position is not None and attitude is not None:
            pose_time = self._sample_time_s(attitude)
            position_time = self._sample_time_s(local_position)
            east = np.interp(pose_time, position_time, local_position.data["y"])
            north = np.interp(pose_time, position_time, local_position.data["x"])
            up = -np.interp(pose_time, position_time, local_position.data["z"])
            wall_time = to_wall(pose_time)
            for index, now in enumerate(wall_time):
                quaternion = tuple(
                    float(attitude.data[f"q[{axis}]"][index]) for axis in range(4)
                )
                roll, pitch, yaw = self._attitude_deg(quaternion)
                self.pose_samples.append(
                    {
                        "wall_time_s": float(now), "sim_time_s": float(pose_time[index]),
                        "x_m": float(east[index]), "y_m": float(north[index]),
                        "z_m": float(up[index]), "roll_deg": roll,
                        "pitch_deg": pitch, "yaw_deg": yaw,
                    }
                )

        if self.pose_samples:
            pose_wall = np.asarray([sample["wall_time_s"] for sample in self.pose_samples])
            origin = self.pose_samples[0]
            along_truth = np.asarray([
                pose["x_m"] - origin["x_m"] for pose in self.pose_samples
            ])
            cross_truth = np.asarray([
                pose["y_m"] - origin["y_m"] for pose in self.pose_samples
            ])
            altitude_truth = np.asarray([
                pose["z_m"] - origin["z_m"] for pose in self.pose_samples
            ])
            pitch_truth = np.asarray([pose["pitch_deg"] for pose in self.pose_samples])
            for sample in live_samples:
                wall = sample["wall_time_s"]
                sample["gazebo_along_track_m"] = float(np.interp(wall, pose_wall, along_truth))
                sample["gazebo_cross_track_m"] = float(np.interp(wall, pose_wall, cross_truth))
                sample["gazebo_altitude_gain_m"] = float(np.interp(wall, pose_wall, altitude_truth))
                sample["gazebo_pitch_deg"] = float(np.interp(wall, pose_wall, pitch_truth))


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
        initial_yaw_deg: float = 0.0,
        standard_plan: Path = DEFAULT_STANDARD_PLAN,
        make_target: str = "gz_honghu_wing_150kg_v8",
        expected_canard_deg: float = 6.0,
        through_touchdown: bool = False,
        touchdown_brake_only: bool = False,
        physics_engine: str | None = None,
        spawn_x_m: float = 0.0,
        spawn_y_m: float = 0.0,
    ) -> None:
        self.scenario = scenario
        self.step_size = step_size
        self.timeout_s = timeout_s
        self.rwto_pitch_ff = rwto_pitch_ff
        self.parameter_overrides = parameter_overrides or {}
        self.cruise_observation_s = cruise_observation_s
        self.mavlink_port = mavlink_port
        self.coincident_takeoff = coincident_takeoff
        self.initial_yaw_deg = initial_yaw_deg
        self.standard_plan = standard_plan
        self.make_target = make_target
        self.expected_canard_deg = expected_canard_deg
        self.through_touchdown = through_touchdown
        self.touchdown_brake_only = touchdown_brake_only
        self.physics_engine = physics_engine
        self.spawn_x_m = spawn_x_m
        self.spawn_y_m = spawn_y_m
        self.snapshot = SitlStateSnapshot()
        self.make_process: subprocess.Popen | None = None
        self.make_log_path: Path | None = None
        self.make_log_file = None
        self.connection = None
        self.aero = OfflineGazeboDiagnostics()
        self.ulog_files_before: set[Path] = set()
        self.target_system = 1
        self.target_component = 1

    def _assert_clean_runtime(self) -> None:
        result = subprocess.run(
            ["ps", "-eo", "pid=,comm=,args="],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        active = []
        for line in result.stdout.splitlines():
            fields = line.strip().split(maxsplit=2)
            if len(fields) < 2:
                continue
            process_id, command = fields[:2]
            arguments = fields[2] if len(fields) == 3 else ""
            if process_id == str(os.getpid()):
                continue
            is_simulator = command in ("px4", "gz", "gzserver")
            is_gz_ruby_launcher = command == "ruby" and "gz sim" in arguments
            if is_simulator or is_gz_ruby_launcher:
                active.append(line.strip())
        if active:
            raise RuntimeError("existing PX4/Gazebo process detected; refusing to disturb it:\n" + "\n".join(active))

    def start(self) -> None:
        self._assert_clean_runtime()
        self.snapshot.capture()
        self.ulog_files_before = set(ROOTFS.glob("log/**/*.ulg"))
        output_dir = ROOT / "analysis_outputs"
        output_dir.mkdir(parents=True, exist_ok=True)
        step_label = f"{self.step_size * 1000:g}".replace(".", "p")
        self.make_log_path = output_dir / f"honghu_v8_{self.scenario}_{step_label}ms.log"
        self.make_log_file = self.make_log_path.open("w", encoding="utf-8")
        env = os.environ.copy()
        env["HEADLESS"] = "1"
        env["PX4_GZ_MAX_STEP_SIZE"] = f"{self.step_size:.7f}"
        if self.physics_engine:
            env["PX4_GZ_PHYSICS_ENGINE"] = self.physics_engine
        # Production 4028 aligns the aircraft with the geographic XY mission.
        # Acceptance missions are deliberately eastbound, so override only the
        # test spawn yaw while retaining the validated 0.5145 m spawn height.
        if self.scenario != "standard":
            env["PX4_GZ_MODEL_POSE"] = (
                f"{self.spawn_x_m:.6f},{self.spawn_y_m:.6f},0.5145,0,0,"
                f"{math.radians(self.initial_yaw_deg):.9f}"
            )
        # Keep automated acceptance independent of a concurrently open QGC,
        # which can otherwise claim both the standard remote endpoint and the
        # PX4 UDP partner learned through local port 18570.
        env["PX4_GCS_LOCAL_PORT"] = str(self.mavlink_port + 3000)
        env["PX4_GCS_REMOTE_PORT"] = str(self.mavlink_port)
        env["PX4_GCS_MINIMAL_STREAMS"] = "1"
        self.make_process = subprocess.Popen(
            ["make", "px4_sitl", self.make_target],
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
        heartbeat = None
        heartbeat_deadline = time.monotonic() + 45.0
        while heartbeat is None and time.monotonic() < heartbeat_deadline:
            heartbeat = self.connection.wait_heartbeat(timeout=1.0)
            if self.make_process.poll() is not None:
                raise RuntimeError(
                    f"PX4/Gazebo target exited before heartbeat; see {self.make_log_path}"
                )
        if heartbeat is None:
            raise TimeoutError(f"PX4 heartbeat not received; see {self.make_log_path}")
        self.target_system = heartbeat.get_srcSystem()
        self.target_component = heartbeat.get_srcComponent()
        self._configure_streams()
        self._set_test_parameters()

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
                # PX4 fixed-wing mission feasibility requires TAKEOFF first
                # and a LAND endpoint. RWTO_TAXI_TEST consumes only TAKEOFF /
                # WAYPOINT coordinates and stops before flight, so these
                # altitudes do not command rotation during this ground test.
                (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0.0, 50.0, 45.0, 12.0),
                (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0.0, 300.0, 45.0, 12.0),
                # Keep the nominal landing glide slope below FW_LND_ANG so
                # Navigator accepts the complete fixed-wing mission.
                (mavutil.mavlink.MAV_CMD_NAV_LAND, 0.0, 1500.0, 0.0, 12.0),
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
        elif self.scenario == "landing":
            specifications = [
                (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0.0, 1000.0, 40.0, 50.0),
                # Landing acceptance validates the approach, touchdown and
                # ground roll rather than climb performance. Keep this handoff
                # at the takeoff item's 40 m altitude: otherwise a low climb
                # rate can make Navigator orbit the waypoint to acquire 50 m,
                # entering LAND hundreds of metres off the final approach.
                (mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0.0, 1600.0, 40.0, 100.0),
                (mavutil.mavlink.MAV_CMD_NAV_LAND, 0.0, 3000.0, 0.0, 100.0),
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

    def upload_standard_plan(self) -> list[dict]:
        """Upload every SimpleItem from the QGC plan without changing geometry."""
        items = load_items(self.standard_plan)
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
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            self._send_gcs_heartbeat()
            message = self.connection.recv_match(
                type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"],
                blocking=True, timeout=1.0,
            )
            if message is None:
                continue
            if message.get_type() == "MISSION_ACK":
                if message.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    raise RuntimeError(f"standard plan rejected with ACK type {message.type}")
                if len(sent) == len(items):
                    self.connection.mav.mission_set_current_send(
                        self.target_system, self.target_component, 0,
                    )
                    return items
                self.connection.mav.mission_count_send(
                    self.target_system, self.target_component, len(items),
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
                )
                continue

            sequence = int(message.seq)
            if not 0 <= sequence < len(items):
                raise RuntimeError(f"PX4 requested invalid standard-plan sequence {sequence}")
            item = items[sequence]
            self.connection.mav.mission_item_int_send(
                self.target_system, self.target_component, sequence,
                item["frame"], item["command"], 1 if sequence == 0 else 0,
                item["autocontinue"], item["param1"], item["param2"],
                item["param3"], item["param4"], item["x"], item["y"], item["z"],
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
            )
            sent.add(sequence)
        raise TimeoutError(f"standard-plan upload timed out after sending {sorted(sent)}")

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

    def collect(self, initial_local: object) -> tuple[list[dict], dict | None, dict | None, float | None]:
        state = TelemetryState(local=initial_local)
        samples: list[dict] = []
        start = time.monotonic()
        last_gcs_heartbeat = 0.0
        rotation = None
        liftoff = None
        liftoff_candidate = None
        loiter_start = None
        loiter_ready_since = None
        landing_start = None
        touchdown_stable_since = None
        touchdown_detected_since = None
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
            sample = {
                "wall_time_s": now,
                "sim_time_s": float(getattr(state.local, "time_boot_ms", 0)) * 1e-3,
                "elapsed_s": now - start,
                "along_track_m": along,
                "cross_track_m": cross,
                "altitude_gain_m": altitude_gain,
                # During flight these aliases use the PX4 estimate only for
                # event scheduling. They are replaced with Gazebo ground truth
                # from the completed ULog before acceptance is evaluated.
                "gazebo_along_track_m": along,
                "gazebo_cross_track_m": cross,
                "gazebo_altitude_gain_m": altitude_gain,
                "vx_m_s": float(state.local.vx),
                "vy_m_s": float(state.local.vy),
                "vz_m_s": float(state.local.vz),
                "groundspeed_m_s": math.hypot(float(state.local.vx), float(state.local.vy)),
                "ground_course_deg": (
                    math.degrees(math.atan2(float(state.local.vy), float(state.local.vx))) % 360.0
                ),
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

            if self.scenario in ("takeoff", "flight", "route", "standard", "landing"):
                if (
                    rotation is None
                    and sample["pitch_deg"] > 2.0
                    and sample["groundspeed_m_s"] > 20.0
                ):
                    rotation = dict(sample)
                if liftoff is None:
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
                        and sample["altitude_gain_m"] >= 47.0:
                    # Canard truth is checked after shutdown from ULog.
                    break
                elif self.scenario == "flight" and loiter_start is None:
                    # Do not force a loiter transition while the takeoff climb
                    # still carries several m/s of vertical speed. Require a
                    # short, genuinely level handoff near the 50 m target so
                    # the test measures normal turning flight rather than a
                    # climb-to-hold step transient.
                    loiter_ready = (
                        47.0 <= sample["altitude_gain_m"] <= 55.0
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
                elif self.scenario == "standard" and state.mission_seq >= 18:
                    if landing_start is None:
                        landing_start = now
                    if self.through_touchdown:
                        if self.touchdown_brake_only:
                            touchdown_detected = (
                                sample["landed_state"]
                                == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
                                or sample["gazebo_altitude_gain_m"] <= 0.55
                            )
                            if touchdown_detected and touchdown_detected_since is None:
                                touchdown_detected_since = now
                            # FW_CANARD_BRKD is 5 s. Retain another 3 s so the
                            # ULog proves the -50 deg brake state without
                            # requiring the simplified wheel model to stop.
                            if (
                                touchdown_detected_since is not None
                                and now - touchdown_detected_since >= 8.0
                            ):
                                break
                        else:
                            touchdown_stable = (
                                sample["landed_state"] == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
                                and sample["groundspeed_m_s"] < 1.0
                                and sample["altitude_gain_m"] < 1.0
                            )
                            if touchdown_stable:
                                touchdown_stable_since = touchdown_stable_since or now
                            else:
                                touchdown_stable_since = None
                            if (
                                touchdown_stable_since is not None
                                and now - touchdown_stable_since >= 2.0
                            ) or now - landing_start >= 90.0:
                                break
                    else:
                        # Retain the approach and first low-altitude segment for
                        # coefficient validation without requiring touchdown.
                        if (
                            sample["altitude_gain_m"] <= 5.0
                            or sample["landed_state"] == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
                            or now - landing_start >= 45.0
                        ):
                            break
                elif self.scenario == "landing" and state.mission_seq >= 2:
                    if landing_start is None:
                        landing_start = now
                    touchdown_stable = (
                        sample["landed_state"] == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
                        and sample["groundspeed_m_s"] < 1.0
                        and sample["altitude_gain_m"] < 1.0
                    )
                    if touchdown_stable:
                        touchdown_stable_since = touchdown_stable_since or now
                    else:
                        touchdown_stable_since = None
                    if (
                        touchdown_stable_since is not None
                        and now - touchdown_stable_since >= 2.0
                    ) or now - landing_start >= 90.0:
                        break
            elif self.scenario == "taxi" and along >= 210.0:
                break

        if not samples:
            raise RuntimeError("no complete telemetry samples collected")
        return samples, rotation, liftoff, loiter_start

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
            target_heading_deg = 90.0
            course_errors = [
                abs((sample["ground_course_deg"] - target_heading_deg + 180.0) % 360.0 - 180.0)
                for sample in samples if sample["groundspeed_m_s"] > 2.0
            ]
            early_count = max(1, min(len(course_errors), int(round(2.0 / 0.05))))
            late_count = max(1, min(len(course_errors), int(round(5.0 / 0.05))))
            initial_course_error = (
                statistics.fmean(course_errors[:early_count]) if course_errors else float("nan")
            )
            final_course_error = (
                statistics.fmean(course_errors[-late_count:]) if course_errors else float("nan")
            )
            steering_commands = self.aero.servo_command_samples.get("servo_8", [])
            metrics = {
                "distance_m": max(sample["along_track_m"] for sample in samples),
                "cross_track_rms_m": rms_cross,
                "cross_track_max_abs_m": max(abs(sample["cross_track_m"]) for sample in steady),
                "steady_groundspeed_mean_m_s": mean_speed,
                "steady_groundspeed_max_m_s": max(sample["groundspeed_m_s"] for sample in steady),
                "altitude_gain_max_m": max(sample["altitude_gain_m"] for sample in samples),
                "roll_max_abs_deg": max(abs(sample["roll_deg"]) for sample in samples),
                "pitch_max_abs_deg": max(abs(sample["pitch_deg"]) for sample in samples),
                "target_heading_deg": target_heading_deg,
                "initial_ground_course_error_abs_deg": initial_course_error,
                "final_ground_course_error_abs_deg": final_course_error,
                "nose_steering_command_max_abs_deg": max(
                    (abs(sample["angle_deg"]) for sample in steering_commands),
                    default=float("nan"),
                ),
            }
            checks = {
                "distance_at_least_200m": metrics["distance_m"] >= 200.0,
                "cross_track_rms_below_0p5m": rms_cross < 0.5,
                "mean_speed_7_to_9m_s": 7.0 <= mean_speed <= 9.0,
                "remained_on_ground": metrics["altitude_gain_max_m"] < 0.5,
                "ground_course_error_not_increased": math.isfinite(final_course_error)
                and math.isfinite(initial_course_error)
                and final_course_error <= initial_course_error + 1.0,
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
                "takeoff_truth_pitch_below_8_5deg": math.isfinite(takeoff_truth_pitch_max)
                and takeoff_truth_pitch_max <= 8.5,
                "climbed_10m": metrics["gazebo_altitude_final_m"] >= 10.0,
                "canards_deployed_at_takeoff": math.isfinite(canard_takeoff_peak_deg)
                and self.expected_canard_deg - 0.5 <= canard_takeoff_peak_deg <= self.expected_canard_deg + 0.5,
                "canard_pair_synchronized": math.isfinite(canard_pair_max_error_deg)
                and canard_pair_max_error_deg < 0.25,
                ("canards_hold_v3_cruise_deflection"
                 if self.expected_canard_deg == 4.0
                 else "canards_hold_expected_cruise_deflection"): math.isfinite(cruise_canard_min_deg)
                and math.isfinite(cruise_canard_max_deg)
                and cruise_canard_min_deg >= self.expected_canard_deg - 0.5
                and cruise_canard_max_deg <= self.expected_canard_deg + 0.5,
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
                "takeoff_truth_pitch_below_8_5deg": math.isfinite(metrics["takeoff_truth_pitch_max_abs_deg"])
                and metrics["takeoff_truth_pitch_max_abs_deg"] <= 8.5,
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
                ("canards_hold_v3_cruise_deflection"
                 if self.expected_canard_deg == 4.0
                 else "canards_hold_expected_cruise_deflection"): math.isfinite(metrics["route_canard_min_deg"])
                and math.isfinite(metrics["route_canard_max_deg"])
                and metrics["route_canard_min_deg"] >= self.expected_canard_deg - 0.5
                and metrics["route_canard_max_deg"] <= self.expected_canard_deg + 0.5,
            }
        elif self.scenario == "landing":
            airborne = [sample for sample in samples if sample["gazebo_altitude_gain_m"] >= 5.0]
            landing_samples = [sample for sample in samples if sample["mission_seq"] >= 2]
            contact_index = next(
                (
                    index for index, sample in enumerate(landing_samples)
                    if sample["gazebo_altitude_gain_m"] <= 0.55
                ),
                None,
            )
            contact = landing_samples[contact_index] if contact_index is not None else None
            post_contact = landing_samples[contact_index:] if contact_index is not None else []
            takeoff_truth = [
                sample for sample in samples
                if sample["mission_seq"] == 0 and sample["gazebo_altitude_gain_m"] <= 20.0
            ]
            metrics = {
                "rotation": rotation,
                "liftoff": liftoff,
                "mission_sequence_max": max(sample["mission_seq"] for sample in samples),
                "takeoff_truth_pitch_max_abs_deg": max(
                    (abs(sample.get("gazebo_pitch_deg", sample["pitch_deg"])) for sample in takeoff_truth),
                    default=float("nan"),
                ),
                "airborne_roll_max_abs_deg": max(
                    (abs(sample["roll_deg"]) for sample in airborne), default=float("nan")
                ),
                "airborne_pitch_max_abs_deg": max(
                    (abs(sample.get("gazebo_pitch_deg", sample["pitch_deg"])) for sample in airborne),
                    default=float("nan"),
                ),
                "touchdown_detected": contact is not None,
                "touchdown_vertical_speed_m_s": contact["vz_m_s"] if contact else float("nan"),
                "touchdown_groundspeed_m_s": contact["groundspeed_m_s"] if contact else float("nan"),
                "post_touchdown_roll_max_abs_deg": max(
                    (abs(sample["roll_deg"]) for sample in post_contact), default=float("nan")
                ),
                "post_touchdown_pitch_max_abs_deg": max(
                    (abs(sample.get("gazebo_pitch_deg", sample["pitch_deg"])) for sample in post_contact),
                    default=float("nan"),
                ),
                "post_touchdown_altitude_min_m": min(
                    (sample["gazebo_altitude_gain_m"] for sample in post_contact), default=float("nan")
                ),
                "final_landed_state": samples[-1]["landed_state"],
                "final_groundspeed_m_s": samples[-1]["groundspeed_m_s"],
                "final_altitude_m": samples[-1]["gazebo_altitude_gain_m"],
            }
            checks = {
                "rotation_detected": rotation is not None,
                "liftoff_detected_from_gazebo_truth": liftoff is not None,
                "takeoff_truth_pitch_below_8_5deg": math.isfinite(
                    metrics["takeoff_truth_pitch_max_abs_deg"]
                ) and metrics["takeoff_truth_pitch_max_abs_deg"] <= 8.5,
                "landing_item_reached": metrics["mission_sequence_max"] >= 2,
                "touchdown_detected": metrics["touchdown_detected"],
                "touchdown_sink_rate_below_1m_s": math.isfinite(
                    metrics["touchdown_vertical_speed_m_s"]
                ) and metrics["touchdown_vertical_speed_m_s"] <= 1.0,
                "touchdown_attitude_remains_upright": math.isfinite(
                    metrics["post_touchdown_roll_max_abs_deg"]
                ) and metrics["post_touchdown_roll_max_abs_deg"] < 15.0
                and metrics["post_touchdown_pitch_max_abs_deg"] < 20.0,
                "touchdown_does_not_fall_through_ground": math.isfinite(
                    metrics["post_touchdown_altitude_min_m"]
                ) and metrics["post_touchdown_altitude_min_m"] > -0.5,
                "landing_stops_and_reports_on_ground":
                metrics["final_landed_state"] == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
                and metrics["final_groundspeed_m_s"] < 1.0,
            }
        elif self.scenario == "standard":
            plan_payload = json.loads(self.standard_plan.read_text(encoding="utf-8-sig"))
            home = plan_payload["mission"]["plannedHomePosition"]
            takeoff_params = plan_payload["mission"]["items"][0]["params"]
            takeoff_north_m = (float(takeoff_params[4]) - float(home[0])) * 111_111.0
            takeoff_east_m = (
                (float(takeoff_params[5]) - float(home[1])) * 111_111.0
                * math.cos(math.radians(float(home[0])))
            )
            runway_heading_rad = math.atan2(takeoff_east_m, takeoff_north_m)
            airborne = [sample for sample in samples if sample["gazebo_altitude_gain_m"] >= 5.0]
            airborne_start = airborne[0]["wall_time_s"] if airborne else float("inf")
            airborne_end = airborne[-1]["wall_time_s"] if airborne else float("-inf")
            airborne_aero = [
                sample for sample in self.aero.samples
                if airborne_start <= sample["wall_time_s"] <= airborne_end
            ]
            takeoff_truth = [
                sample for sample in samples
                if sample["gazebo_altitude_gain_m"] <= 20.0 and sample["mission_seq"] == 0
            ]
            ground_samples = [
                sample for sample in samples
                if liftoff is not None and sample["wall_time_s"] <= liftoff["wall_time_s"]
            ]
            runway_cross_track = [
                -math.sin(runway_heading_rad) * sample["gazebo_cross_track_m"]
                + math.cos(runway_heading_rad) * sample["gazebo_along_track_m"]
                for sample in ground_samples
            ]
            canard_values = [
                value
                for aero in airborne_aero
                for value in (
                    aero["delta_doc_deg"][3], aero["theta_joint_deg"][6],
                    aero["theta_joint_deg"][7],
                )
            ]
            metrics = {
                "rotation": rotation,
                "liftoff": liftoff,
                "mission_sequence_max": max(sample["mission_seq"] for sample in samples),
                "elapsed_wall_s": samples[-1]["elapsed_s"],
                "elapsed_sim_s": samples[-1]["sim_time_s"] - samples[0]["sim_time_s"],
                "airborne_sample_count": len(airborne),
                "aero_truth_sample_count": len(airborne_aero),
                "runway_heading_deg": math.degrees(runway_heading_rad),
                "ground_cross_track_max_abs_m": max(
                    (abs(value) for value in runway_cross_track),
                    default=float("nan"),
                ),
                "takeoff_truth_pitch_max_abs_deg": max(
                    (abs(sample.get("gazebo_pitch_deg", sample["pitch_deg"])) for sample in takeoff_truth),
                    default=float("nan"),
                ),
                "airborne_altitude_min_m": min(
                    (sample["gazebo_altitude_gain_m"] for sample in airborne), default=float("nan")
                ),
                "airborne_altitude_max_m": max(
                    (sample["gazebo_altitude_gain_m"] for sample in airborne), default=float("nan")
                ),
                "airborne_airspeed_min_m_s": min(
                    (sample["airspeed_m_s"] for sample in airborne), default=float("nan")
                ),
                "airborne_airspeed_max_m_s": max(
                    (sample["airspeed_m_s"] for sample in airborne), default=float("nan")
                ),
                "airborne_roll_max_abs_deg": max(
                    (abs(sample["roll_deg"]) for sample in airborne), default=float("nan")
                ),
                "airborne_pitch_max_abs_deg": max(
                    (abs(sample.get("gazebo_pitch_deg", sample["pitch_deg"])) for sample in airborne),
                    default=float("nan"),
                ),
                "airborne_alpha_range_deg": [
                    min((aero["alpha_deg"] for aero in airborne_aero), default=float("nan")),
                    max((aero["alpha_deg"] for aero in airborne_aero), default=float("nan")),
                ],
                "airborne_beta_range_deg": [
                    min((aero["beta_deg"] for aero in airborne_aero), default=float("nan")),
                    max((aero["beta_deg"] for aero in airborne_aero), default=float("nan")),
                ],
                "airborne_canard_min_deg": min(canard_values, default=float("nan")),
                "airborne_canard_max_deg": max(canard_values, default=float("nan")),
            }
            if self.through_touchdown:
                landing_samples = [sample for sample in samples if sample["mission_seq"] >= 18]
                contact_index = next(
                    (
                        index for index, sample in enumerate(landing_samples)
                        if sample["gazebo_altitude_gain_m"] <= 0.55
                    ),
                    None,
                )
                contact = landing_samples[contact_index] if contact_index is not None else None
                post_contact = landing_samples[contact_index:] if contact_index is not None else []
                metrics.update({
                    "touchdown_detected": contact is not None,
                    "touchdown_vertical_speed_m_s": contact["vz_m_s"] if contact else float("nan"),
                    "touchdown_groundspeed_m_s": contact["groundspeed_m_s"] if contact else float("nan"),
                    "post_touchdown_roll_max_abs_deg": max(
                        (abs(sample["roll_deg"]) for sample in post_contact), default=float("nan")
                    ),
                    "post_touchdown_pitch_max_abs_deg": max(
                        (abs(sample.get("gazebo_pitch_deg", sample["pitch_deg"])) for sample in post_contact),
                        default=float("nan"),
                    ),
                    "post_touchdown_altitude_min_m": min(
                        (sample["gazebo_altitude_gain_m"] for sample in post_contact), default=float("nan")
                    ),
                    "final_landed_state": samples[-1]["landed_state"],
                    "final_groundspeed_m_s": samples[-1]["groundspeed_m_s"],
                    "final_altitude_m": samples[-1]["gazebo_altitude_gain_m"],
                    "post_touchdown_record_duration_s": (
                        samples[-1]["wall_time_s"] - contact["wall_time_s"]
                        if contact else float("nan")
                    ),
                })
            checks = {
                "rotation_detected": rotation is not None,
                "liftoff_detected_from_gazebo_truth": liftoff is not None,
                "liftoff_before_45m_s": liftoff is not None and liftoff["airspeed_m_s"] <= 45.0,
                "takeoff_truth_pitch_below_8_5deg": math.isfinite(
                    metrics["takeoff_truth_pitch_max_abs_deg"]
                ) and metrics["takeoff_truth_pitch_max_abs_deg"] <= 8.5,
                "ground_cross_track_below_3m": math.isfinite(
                    metrics["ground_cross_track_max_abs_m"]
                ) and metrics["ground_cross_track_max_abs_m"] < 3.0,
                "reached_plan_landing_item_18": metrics["mission_sequence_max"] >= 18,
                "airborne_truth_available": len(airborne_aero) >= 1000,
                "airborne_attitude_bounded": math.isfinite(metrics["airborne_roll_max_abs_deg"])
                and metrics["airborne_roll_max_abs_deg"] < 35.0
                and metrics["airborne_pitch_max_abs_deg"] < 15.0,
                "airborne_airspeed_bounded": math.isfinite(metrics["airborne_airspeed_min_m_s"])
                and metrics["airborne_airspeed_min_m_s"] >= 25.0
                and metrics["airborne_airspeed_max_m_s"] <= 60.0,
                ("canards_hold_v3_airborne_deflection"
                 if self.expected_canard_deg == 4.0
                 else "canards_hold_expected_airborne_deflection"): math.isfinite(
                    metrics["airborne_canard_min_deg"]
                ) and metrics["airborne_canard_min_deg"] >= self.expected_canard_deg - 0.5
                and metrics["airborne_canard_max_deg"] <= self.expected_canard_deg + 0.5,
            }
            if self.through_touchdown:
                touchdown_checks = {
                    "touchdown_detected": metrics["touchdown_detected"],
                    "touchdown_sink_rate_below_1m_s": math.isfinite(
                        metrics["touchdown_vertical_speed_m_s"]
                    ) and metrics["touchdown_vertical_speed_m_s"] <= 1.0,
                    "touchdown_attitude_remains_upright": math.isfinite(
                        metrics["post_touchdown_roll_max_abs_deg"]
                    ) and metrics["post_touchdown_roll_max_abs_deg"] < 15.0
                    and metrics["post_touchdown_pitch_max_abs_deg"] < 20.0,
                    "touchdown_does_not_fall_through_ground": math.isfinite(
                        metrics["post_touchdown_altitude_min_m"]
                    ) and metrics["post_touchdown_altitude_min_m"] > -0.5,
                }
                if self.touchdown_brake_only:
                    touchdown_checks["post_touchdown_record_covers_canard_brake"] = (
                        math.isfinite(metrics["post_touchdown_record_duration_s"])
                        and metrics["post_touchdown_record_duration_s"] >= 7.5
                    )
                else:
                    touchdown_checks["landing_stops_and_reports_on_ground"] = (
                        metrics["final_landed_state"]
                        == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
                        and metrics["final_groundspeed_m_s"] < 1.0
                    )
                checks.update(touchdown_checks)
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
                "takeoff_truth_pitch_below_8_5deg": math.isfinite(takeoff_truth_pitch_max)
                and takeoff_truth_pitch_max <= 8.5,
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
                ("canards_hold_v3_cruise_deflection"
                 if self.expected_canard_deg == 4.0
                 else "canards_hold_expected_cruise_deflection"): math.isfinite(metrics["cruise_canard_min_deg"])
                and math.isfinite(metrics["cruise_canard_max_deg"])
                and metrics["cruise_canard_min_deg"] >= self.expected_canard_deg - 0.5
                and metrics["cruise_canard_max_deg"] <= self.expected_canard_deg + 0.5,
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
        mission: list[dict] = []
        collected: tuple[list[dict], dict | None, dict | None, float | None] | None = None
        try:
            self.start()
            global_position, local_position = self.wait_for_position()
            if self.scenario == "static":
                mission = []
            else:
                mission = self.upload_standard_plan() if self.scenario == "standard" \
                    else self.upload_eastbound_mission(global_position)
                self.arm_and_start_mission()
            collected = self.collect(local_position)
        finally:
            self.stop()

        if collected is None:
            raise RuntimeError("dynamic run stopped before telemetry collection completed")
        samples, rotation, liftoff, loiter_start = collected
        new_logs = sorted(
            set(ROOTFS.glob("log/**/*.ulg")) - self.ulog_files_before,
            key=lambda path: path.stat().st_mtime,
        )
        if not new_logs:
            raise RuntimeError("PX4 stopped without producing a new ULog for offline truth analysis")
        self.aero.load(new_logs[-1], samples)

        # Re-detect the events after the PX4-estimator aliases have been
        # replaced with ULog Gazebo truth. No diagnostic process was active
        # while the flight-control loop was running.
        rotation = next(
            (
                dict(sample) for sample in samples
                if sample.get("gazebo_pitch_deg", sample["pitch_deg"]) > 2.0
                and sample["groundspeed_m_s"] > 20.0
            ),
            rotation,
        )
        liftoff = None
        liftoff_candidate = None
        for sample in samples:
            altitude = sample["gazebo_altitude_gain_m"]
            if liftoff_candidate is None and altitude > 0.5 and sample["groundspeed_m_s"] > 20.0:
                liftoff_candidate = dict(sample)
            elif liftoff_candidate is not None and altitude < 0.25:
                liftoff_candidate = None
            if liftoff_candidate is not None and altitude > 2.0:
                liftoff = liftoff_candidate
                break

        report = self._evaluate(samples, rotation, liftoff, loiter_start)
        report["mission"] = mission
        report["make_log"] = str(self.make_log_path)
        report["offline_ulog"] = str(new_logs[-1])
        report["diagnostic_mode"] = "post-flight ULog only; no external Gazebo topic observers"
        if self.scenario == "standard":
            report["plan"] = str(self.standard_plan)
            # The ULog is the authoritative full-rate artifact. Avoid writing
            # tens of thousands of duplicated dictionaries to the JSON report.
            report["samples"] = []
            report["aero_samples"] = []
            report["propulsion_samples"] = []
            report["gazebo_pose_samples"] = []
            report["offline_series_storage"] = "ULog"
        return report

    def stop(self) -> None:
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
    parser.add_argument(
        "scenario", choices=("static", "taxi", "takeoff", "flight", "route", "landing", "standard")
    )
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
    parser.add_argument(
        "--initial-yaw-deg", type=float, default=0.0,
        help="Gazebo ENU spawn yaw [deg], useful for a taxi steering disturbance test",
    )
    parser.add_argument(
        "--plan", type=Path, default=DEFAULT_STANDARD_PLAN,
        help="QGroundControl .plan used by the standard scenario",
    )
    parser.add_argument(
        "--make-target", default="gz_honghu_wing_150kg_v8",
        help="SITL make target; defaults to the stable production V8",
    )
    parser.add_argument(
        "--expected-canard-deg", type=float, default=6.0,
        help="expected airborne canard angle used only by acceptance checks",
    )
    parser.add_argument(
        "--through-touchdown", action="store_true",
        help="for the standard plan, continue through touchdown and require a stable stop",
    )
    parser.add_argument(
        "--touchdown-brake-only", action="store_true",
        help=(
            "for the standard plan, record 8 s after touchdown so the canard "
            "brake transition is captured without requiring a full stop"
        ),
    )
    parser.add_argument(
        "--physics-engine",
        choices=("gz-physics-dartsim-plugin", "gz-physics-bullet-featherstone-plugin"),
        help="temporary Gazebo physics-engine plugin; omitted keeps the normal DART default",
    )
    parser.add_argument("--spawn-x", type=float, default=0.0, help="test-only Gazebo ENU spawn X [m]")
    parser.add_argument("--spawn-y", type=float, default=0.0, help="test-only Gazebo ENU spawn Y [m]")
    parser.add_argument("--no-assert", action="store_true", help="write measurements without a failing exit code")
    arguments = parser.parse_args()
    if not 1024 <= arguments.mavlink_port <= 62535:
        parser.error("--mavlink-port must be in 1024..62535")
    if arguments.coincident_takeoff and arguments.scenario not in ("takeoff", "flight"):
        parser.error("--coincident-takeoff is supported only by takeoff and flight scenarios")
    if arguments.scenario != "standard" and abs(arguments.initial_yaw_deg) > 30.0:
        parser.error("--initial-yaw-deg must be within +/-30 degrees")
    arguments.plan = arguments.plan.resolve()
    if arguments.scenario == "standard" and not arguments.plan.exists():
        parser.error(f"standard plan does not exist: {arguments.plan}")
    if (arguments.through_touchdown or arguments.touchdown_brake_only) \
            and arguments.scenario != "standard":
        parser.error("touchdown continuation is supported only by the standard scenario")
    if arguments.through_touchdown and arguments.touchdown_brake_only:
        parser.error("--through-touchdown and --touchdown-brake-only are mutually exclusive")
    if not arguments.make_target.startswith("gz_honghu_wing_"):
        parser.error("--make-target must be a Honghu Gazebo target")
    default_timeouts = {
        "static": 62.0,
        "taxi": 70.0,
        "takeoff": 110.0,
        "flight": 150.0,
        "route": 190.0,
        "landing": 300.0,
        "standard": 1000.0,
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
        arguments.initial_yaw_deg,
        arguments.plan,
        arguments.make_target,
        arguments.expected_canard_deg,
        arguments.through_touchdown or arguments.touchdown_brake_only,
        arguments.touchdown_brake_only,
        arguments.physics_engine,
        arguments.spawn_x,
        arguments.spawn_y,
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
