#!/usr/bin/env python3
"""Deterministic Gazebo runtime acceptance for Honghu Wing V8 aerodynamics.

The fixture locks base_link to world while retaining all real control joints.
A 30 m/s uniform headwind therefore exercises the production Gazebo plugin and
joint controllers without attitude drift or a hidden velocity controller.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import tempfile
import time
from pathlib import Path

from compare_honghu_v8_runtime import compare


ROOT = Path(__file__).resolve().parents[2]
MODEL_DIR = ROOT / "simulation_models/models/honghu_wing_150kg_v8"
PLUGIN_DIR = ROOT / "build/px4_sitl_default/src/modules/simulation/gz_plugins"
MODEL_NAME = "honghu_runtime"
WORLD_NAME = "honghu_aero_runtime"
AERO_TOPIC = f"/model/{MODEL_NAME}/honghu_v8/aero_state"
MOMENT_TOPIC = f"/model/{MODEL_NAME}/honghu_v8/moment_gz_flu"
JOINT_TOPIC = f"/world/{WORLD_NAME}/model/{MODEL_NAME}/joint_state"


def run(command: list[str], env: dict[str, str], timeout: float = 20.0) -> str:
    return subprocess.run(
        command, env=env, check=True, text=True, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, timeout=timeout,
    ).stdout


def read_json_messages(topic: str, count: int, env: dict[str, str]) -> list[dict]:
    output = run(
        ["gz", "topic", "-e", "-n", str(count), "--json-output", "-t", topic],
        env, timeout=20.0,
    )
    return [json.loads(line) for line in output.splitlines() if line.strip()]


def read_aero_frames(count: int, env: dict[str, str]) -> list[list[float]]:
    messages = read_json_messages(AERO_TOPIC, count, env)
    frames = [message["data"] for message in messages]
    if len(frames) != count or any(len(frame) != 76 for frame in frames):
        raise RuntimeError("incomplete aero_state diagnostic frame")
    return frames


def read_moment(env: dict[str, str]) -> tuple[float, float, float]:
    message = read_json_messages(MOMENT_TOPIC, 1, env)[0]
    return tuple(float(message.get(axis, 0.0)) for axis in ("x", "y", "z"))


def publish_double(topic: str, value: float, env: dict[str, str]) -> None:
    run(
        ["gz", "topic", "-t", topic, "-m", "gz.msgs.Double", "-p", f"data: {value:.17g}"],
        env,
    )


def command_controls(
    theta_deg: tuple[float, ...], env: dict[str, str], settle_s: float = 0.15
) -> None:
    for index, value in enumerate(theta_deg):
        publish_double(
            f"/model/{MODEL_NAME}/servo_{index}", math.radians(value), env
        )
    time.sleep(settle_s)


def stable_joint_angles(
    frames: list[list[float]], target_deg: tuple[float, ...], tolerance_deg: float = 0.05
) -> dict[str, float]:
    result = {}
    for index, target in enumerate(target_deg):
        values = [frame[15 + index] for frame in frames]
        mean = sum(values) / len(values)
        span = max(values) - min(values)
        if abs(mean - target) > tolerance_deg:
            raise AssertionError(
                f"servo_{index} endpoint error: {mean:.6f} vs {target:.6f} deg"
            )
        if span > 0.01:
            raise AssertionError(
                f"servo_{index} endpoint oscillation: {span:.6g} deg over sampled frames"
            )
        result[f"servo_{index}"] = mean
    return result


def endpoint_deflections(
    frame: list[float], target_deg: tuple[float, float, float, float], tolerance_deg: float = 0.05
) -> dict[str, float]:
    result = {}
    for index, (name, target) in enumerate(zip(("a", "e", "r", "c"), target_deg)):
        value = frame[23 + index]
        if abs(value - target) > tolerance_deg:
            raise AssertionError(
                f"delta_{name} endpoint error: {value:.6f} vs {target:.6f} deg"
            )
        result[name] = value
    return result


def stable_deflections(frames: list[list[float]], target: float) -> dict[str, float]:
    result = {}
    for index, name in enumerate(("a", "e", "r", "c")):
        values = [frame[23 + index] for frame in frames]
        mean = sum(values) / len(values)
        span = max(values) - min(values)
        if abs(mean - target) > 0.02:
            raise AssertionError(f"delta_{name} target error: {mean:.6f} vs {target:.6f} deg")
        if span > 1e-4:
            raise AssertionError(f"delta_{name} oscillation: {span:.6g} deg over sampled frames")
        result[name] = mean
    return result


def contribution_signs(frame: list[float], expected: float) -> None:
    samples = {
        "aileron_Cl": frame[27 + 3],
        "elevator_Cm": frame[33 + 4],
        "rudder_Cn": frame[39 + 5],
        "canard_Cm": frame[45 + 4],
    }
    for name, value in samples.items():
        if value * expected <= 0.0:
            raise AssertionError(f"wrong runtime control sign {name}={value:+.9g}")


def moment_increment_signs(
    baseline: tuple[float, float, float],
    controlled: tuple[float, float, float],
    expected: float,
) -> dict[str, float]:
    delta = tuple(controlled[i] - baseline[i] for i in range(3))
    # Positive document deflections: +GZ Mx, -GZ My, -GZ Mz.
    if delta[0] * expected <= 0.0 or delta[1] * expected >= 0.0 or delta[2] * expected >= 0.0:
        raise AssertionError(f"wrong final Gazebo moment increment: {delta}")
    return dict(zip(("Mx", "My", "Mz"), delta))


def check_nose_steering(env: dict[str, str]) -> dict[str, float]:
    publish_double(f"/model/{MODEL_NAME}/servo_8", math.radians(1.0), env)
    time.sleep(0.15)
    messages = read_json_messages(JOINT_TOPIC, 3, env)
    positions = []
    velocities = []
    for message in messages:
        joint = next(item for item in message["joint"] if item["name"] == "nose_steering_joint")
        axis = joint["axis1"]
        positions.append(math.degrees(float(axis.get("position", 0.0))))
        velocities.append(float(axis.get("velocity", 0.0)))
    mean = sum(positions) / len(positions)
    if abs(mean - 1.0) > 0.02:
        raise AssertionError(f"nose steering target error: {mean:.6f} deg")
    if max(positions) - min(positions) > 1e-4 or max(abs(v) for v in velocities) > 1e-3:
        raise AssertionError(f"nose steering oscillation: positions={positions}, velocities={velocities}")
    return {"position_deg": mean, "max_abs_velocity_rad_s": max(abs(v) for v in velocities)}


def write_fixture(directory: Path, step_size_s: float) -> Path:
    model = (MODEL_DIR / "model.sdf").read_text(encoding="utf-8")
    model = model.replace("<uri>meshes/", f"<uri>{MODEL_DIR / 'meshes'}/")
    lock = (
        '    <joint name="runtime_fixture_lock" type="fixed">'
        '<parent>world</parent><child>base_link</child></joint>\n'
    )
    marker = "  </model>\n</sdf>"
    if model.count(marker) != 1:
        raise RuntimeError("cannot inject runtime fixture lock")
    model = model.replace(marker, lock + marker)
    model_path = directory / "model.sdf"
    model_path.write_text(model, encoding="utf-8")

    world = f"""<sdf version="1.10">
  <world name="{WORLD_NAME}">
    <physics name="aero_test" type="ignored"><max_step_size>{step_size_s:.9g}</max_step_size><real_time_factor>0</real_time_factor></physics>
    <gravity>0 0 0</gravity><magnetic_field>0 0 0</magnetic_field>
    <wind><linear_velocity>-30 0 0</linear_velocity></wind>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <include><uri>{model_path}</uri><name>{MODEL_NAME}</name><pose>0 0 100 0 0 0</pose></include>
  </world>
</sdf>
"""
    world_path = directory / "world.sdf"
    world_path.write_text(world, encoding="utf-8")
    return world_path


def wait_for_topic(env: dict[str, str], server: subprocess.Popen, timeout: float = 20.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if server.poll() is not None:
            raise RuntimeError(f"Gazebo server exited with {server.returncode}")
        try:
            if AERO_TOPIC in run(["gz", "topic", "-l"], env, timeout=3.0).splitlines():
                return
        except (subprocess.SubprocessError, OSError):
            pass
        time.sleep(0.1)
    raise TimeoutError(f"Gazebo topic did not appear: {AERO_TOPIC}")


def execute(output_path: Path | None, step_size_s: float = 0.001) -> dict:
    env = os.environ.copy()
    env["GZ_PARTITION"] = f"honghu_v8_aero_test_{os.getpid()}"
    env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
        value for value in (str(ROOT / "simulation_models/models"), env.get("GZ_SIM_RESOURCE_PATH", "")) if value
    )
    env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.pathsep.join(
        value for value in (str(PLUGIN_DIR), env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")) if value
    )

    with tempfile.TemporaryDirectory(prefix="honghu_v8_aero_") as temp:
        directory = Path(temp)
        world = write_fixture(directory, step_size_s)
        with (directory / "gz_server.log").open("w", encoding="utf-8") as log:
            server = subprocess.Popen(
                ["gz", "sim", "-r", "-s", str(world)], env=env,
                stdout=log, stderr=subprocess.STDOUT, text=True,
            )
            try:
                wait_for_topic(env, server)
                baseline_frame = read_aero_frames(1, env)[0]
                baseline_moment = read_moment(env)
                baseline_compare = compare(baseline_frame, 2e-10)

                positive_theta = (-1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
                command_controls(positive_theta, env)
                positive_frames = read_aero_frames(3, env)
                positive_delta = stable_deflections(positive_frames, 1.0)
                contribution_signs(positive_frames[-1], 1.0)
                positive_compare = compare(positive_frames[-1], 2e-10)
                positive_moment = moment_increment_signs(baseline_moment, read_moment(env), 1.0)

                negative_theta = (1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0)
                command_controls(negative_theta, env)
                negative_frames = read_aero_frames(3, env)
                negative_delta = stable_deflections(negative_frames, -1.0)
                contribution_signs(negative_frames[-1], -1.0)
                negative_compare = compare(negative_frames[-1], 2e-10)
                negative_moment = moment_increment_signs(baseline_moment, read_moment(env), -1.0)

                # The small-angle checks above validate signs and table math.
                # Also exercise the complete mechanical ranges: takeoff needs a
                # large elevator step, while the stage-only canards have the
                # asymmetric -50/+15 degree range specified by the V8 contract.
                positive_endpoints = (-30.0, 30.0, 30.0, 30.0, 30.0, 30.0, 15.0, 15.0)
                command_controls(positive_endpoints, env, settle_s=0.3)
                positive_endpoint_frames = read_aero_frames(3, env)
                positive_endpoint_tracking = stable_joint_angles(
                    positive_endpoint_frames, positive_endpoints
                )
                positive_endpoint_delta = endpoint_deflections(
                    positive_endpoint_frames[-1], (30.0, 30.0, 30.0, 15.0)
                )

                negative_endpoints = (30.0, -30.0, -30.0, -30.0, -30.0, -30.0, -50.0, -50.0)
                command_controls(negative_endpoints, env, settle_s=0.5)
                negative_endpoint_frames = read_aero_frames(3, env)
                negative_endpoint_tracking = stable_joint_angles(
                    negative_endpoint_frames, negative_endpoints
                )
                negative_endpoint_delta = endpoint_deflections(
                    negative_endpoint_frames[-1], (-30.0, -30.0, -30.0, -50.0)
                )
                nose = check_nose_steering(env)

                report = {
                    "status": "PASS",
                    "fixture": "base_link fixed to world; real V8 joints; 30 m/s headwind",
                    "step_size_s": step_size_s,
                    "baseline_runtime_offline_max_error": baseline_compare["maximum_absolute_error"],
                    "positive": {
                        "delta_doc_deg": positive_delta,
                        "moment_increment_gz_flu_nm": positive_moment,
                        "runtime_offline_max_error": positive_compare["maximum_absolute_error"],
                    },
                    "negative": {
                        "delta_doc_deg": negative_delta,
                        "moment_increment_gz_flu_nm": negative_moment,
                        "runtime_offline_max_error": negative_compare["maximum_absolute_error"],
                    },
                    "mechanical_endpoints": {
                        "positive_joint_deg": positive_endpoint_tracking,
                        "positive_delta_doc_deg": positive_endpoint_delta,
                        "negative_joint_deg": negative_endpoint_tracking,
                        "negative_delta_doc_deg": negative_endpoint_delta,
                    },
                    "nose_steering": nose,
                }
            finally:
                try:
                    run(
                        ["gz", "service", "-s", "/server_control", "--reqtype", "gz.msgs.ServerControl",
                         "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", "stop: true"],
                        env, timeout=5.0,
                    )
                except (subprocess.SubprocessError, OSError):
                    server.terminate()
                try:
                    server.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    server.kill()
                    server.wait(timeout=5.0)

    if output_path:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", type=Path)
    parser.add_argument("--step-size", type=float, default=0.001)
    args = parser.parse_args()
    if args.step_size <= 0.0 or args.step_size > 0.002:
        parser.error("--step-size must be in (0, 0.002]")
    report = execute(args.json, args.step_size)
    print(f"Honghu V8 Gazebo aerodynamic runtime acceptance: PASS ({report['step_size_s']*1000:g} ms)")
    print(f"  positive delta_doc={report['positive']['delta_doc_deg']}")
    print(f"  negative delta_doc={report['negative']['delta_doc_deg']}")
    print(f"  nose steering={report['nose_steering']['position_deg']:.6f} deg, no sampled oscillation")
    print("  runtime/offline coefficients and final Gazebo moment increments: PASS")


if __name__ == "__main__":
    main()
