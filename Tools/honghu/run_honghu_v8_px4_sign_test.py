#!/usr/bin/env python3
"""PX4 -> allocator -> Gazebo joint -> aerodynamic moment sign acceptance.

Prerequisite: PX4 SITL is connected to the fixed 30 m/s fixture described in
the V8 README.  The script keeps MANUAL_CONTROL alive, commands Stabilized
mode, and compares each positive PX4 torque increment with delta_doc and the
final Gazebo FLU moment increment.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import threading
import time
from pathlib import Path

from pymavlink import mavutil

from compare_honghu_v8_runtime import compare


ROOT = Path(__file__).resolve().parents[2]
PX4_BIN = ROOT / "build/px4_sitl_default/bin"
MODEL = "honghu_px4_fixture"
AERO_TOPIC = f"/model/{MODEL}/honghu_v8/aero_state"
MOMENT_TOPIC = f"/model/{MODEL}/honghu_v8/moment_gz_flu"


def run(command: list[str], timeout: float = 15.0) -> str:
    return subprocess.run(
        command, check=True, text=True, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, timeout=timeout,
    ).stdout


def read_json_topic(topic: str) -> dict:
    output = run(["gz", "topic", "-e", "-n", "1", "--json-output", "-t", topic])
    return json.loads(output.strip())


def read_torque() -> tuple[float, float, float]:
    output = run([str(PX4_BIN / "px4-listener"), "vehicle_torque_setpoint", "-n", "1"])
    match = re.search(r"xyz:\s*\[([^]]+)\]", output)
    if not match:
        raise RuntimeError(f"cannot parse vehicle_torque_setpoint:\n{output}")
    values = tuple(float(value.strip()) for value in match.group(1).split(","))
    if len(values) != 3:
        raise RuntimeError(f"invalid vehicle_torque_setpoint: {values}")
    return values


def snapshot() -> dict:
    torque = read_torque()
    frame = read_json_topic(AERO_TOPIC)["data"]
    moment_message = read_json_topic(MOMENT_TOPIC)
    moment = tuple(float(moment_message.get(axis, 0.0)) for axis in ("x", "y", "z"))
    truth = compare(frame, 2e-10)
    return {
        "torque_setpoint": torque,
        "delta_doc_deg": tuple(frame[23:27]),
        "moment_gz_flu_nm": moment,
        "runtime_offline_max_error": truth["maximum_absolute_error"],
    }


class ManualStream:
    def __init__(self) -> None:
        self.connection = mavutil.mavlink_connection(
            "udpin:127.0.0.1:14550", source_system=250
        )
        heartbeat = self.connection.wait_heartbeat(timeout=10)
        if heartbeat is None:
            raise TimeoutError("PX4 MAVLink heartbeat not received on UDP 14550")
        self.target = heartbeat.get_srcSystem()
        self.command = (0, 0, 0, 0)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        while not self.stop_event.is_set():
            x, y, z, r = self.command
            self.connection.mav.manual_control_send(self.target, x, y, z, r, 0)
            time.sleep(0.02)

    def start(self) -> None:
        self.thread.start()

    def set(self, x: int = 0, y: int = 0, z: int = 0, r: int = 0) -> None:
        self.command = (x, y, z, r)

    def close(self) -> None:
        self.set()
        time.sleep(0.2)
        self.stop_event.set()
        self.thread.join(timeout=2.0)
        self.connection.close()


def increment(controlled: tuple[float, ...], baseline: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(controlled[index] - baseline[index] for index in range(len(baseline)))


def execute(output_path: Path | None) -> dict:
    stream = ManualStream()
    stream.start()
    try:
        time.sleep(1.0)
        run([str(PX4_BIN / "px4-commander"), "mode", "stabilized"])
        run([str(PX4_BIN / "px4-commander"), "arm", "-f"])
        time.sleep(1.0)
        baseline = snapshot()

        cases = {
            "roll": {"manual": {"y": 350}, "torque_axis": 0, "delta_axis": 0, "moment_axis": 0, "moment_sign": 1},
            # MAVLink +X stick is nose-down; pull-back (-X) requests +FRD pitch.
            "pitch": {"manual": {"x": -250}, "torque_axis": 1, "delta_axis": 1, "moment_axis": 1, "moment_sign": -1},
            "yaw": {"manual": {"r": 350}, "torque_axis": 2, "delta_axis": 2, "moment_axis": 2, "moment_sign": -1},
        }
        results = {}
        for name, case in cases.items():
            stream.set(**case["manual"])
            time.sleep(1.0)
            controlled = snapshot()
            torque_delta = increment(controlled["torque_setpoint"], baseline["torque_setpoint"])
            delta_doc = increment(controlled["delta_doc_deg"], baseline["delta_doc_deg"])
            moment_delta = increment(controlled["moment_gz_flu_nm"], baseline["moment_gz_flu_nm"])
            if torque_delta[case["torque_axis"]] <= 0.01:
                raise AssertionError(f"{name}: positive PX4 torque was not produced: {torque_delta}")
            if delta_doc[case["delta_axis"]] <= 0.05:
                raise AssertionError(f"{name}: positive delta_doc was not produced: {delta_doc}")
            if moment_delta[case["moment_axis"]] * case["moment_sign"] <= 0.5:
                raise AssertionError(f"{name}: wrong final Gazebo moment increment: {moment_delta}")
            results[name] = {
                "manual_control": case["manual"],
                "vehicle_torque_setpoint_increment": torque_delta,
                "delta_doc_increment_deg": delta_doc,
                "moment_gz_flu_increment_nm": moment_delta,
                "runtime_offline_max_error": controlled["runtime_offline_max_error"],
            }
            stream.set()
            time.sleep(0.6)

        report = {
            "status": "PASS",
            "fixture": "PX4 Stabilized; base fixed; 30 m/s headwind; real allocator/bridge/joints/aero",
            "baseline": baseline,
            "cases": results,
        }
    finally:
        try:
            run([str(PX4_BIN / "px4-commander"), "disarm", "-f"], timeout=5.0)
        except (subprocess.SubprocessError, OSError):
            pass
        stream.close()

    if output_path:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    report = execute(args.json)
    print("Honghu V8 PX4-to-aerodynamic-moment sign acceptance: PASS")
    for name, case in report["cases"].items():
        print(
            f"  {name}: torque_delta={case['vehicle_torque_setpoint_increment']}, "
            f"delta_doc={case['delta_doc_increment_deg']}, "
            f"moment_GZ={case['moment_gz_flu_increment_nm']}"
        )


if __name__ == "__main__":
    main()
