#!/usr/bin/env python3
"""Safe airborne pitch-loop identification for the Honghu V8 100 kg model.

The aircraft performs its normal runway takeoff using the isolated 4038
airframe.  After reaching a modified 120 m version of the classic mission, the
script temporarily enters OFFBOARD and excites either the pitch-rate loop or
the pitch-attitude loop.  PX4 parameter/dataman files are restored by the
shared DynamicRun harness after every run.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import tempfile
import time
from pathlib import Path

import numpy as np
from pyulog import ULog
from pymavlink import mavutil

from analyze_honghu_v8_longitudinal_control import transfer_at, welch_cross_spectrum
from run_honghu_v8_dynamic_acceptance import DynamicRun, PX4_BIN, ROOTFS, run


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PLAN = Path("/home/fly/px4_reference_docs/current/模仿XY航线规划.plan")

RATE_STEPS = (
    (0.0, 5.0),
    (2.0, 1.0),
    (-2.0, 1.0),
    (0.0, 4.0),
    (-2.0, 1.0),
    (2.0, 1.0),
    (0.0, 4.0),
    (4.0, 0.5),
    (-4.0, 0.5),
    (0.0, 4.0),
    (-4.0, 0.5),
    (4.0, 0.5),
    (0.0, 4.0),
)

ATTITUDE_STEPS = (
    (0.0, 5.0),
    (1.5, 6.0),
    (0.0, 6.0),
    (-1.5, 6.0),
    (0.0, 6.0),
    (3.0, 6.0),
    (0.0, 6.0),
    (-3.0, 6.0),
    (0.0, 6.0),
)

MULTISINE_FREQUENCIES_HZ = (0.15, 0.23, 0.4, 0.7, 1.0, 1.5)


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> list[float]:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def pitch_from_quaternion(quaternion) -> float:
    w, x, y, z = (float(value) for value in quaternion)
    return math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))


def parse_parameter_overrides(values: list[str]) -> dict[str, float]:
    result = {}
    for specification in values:
        if "=" not in specification:
            raise ValueError(f"invalid --param {specification!r}; expected NAME=VALUE")
        name, raw_value = specification.split("=", 1)
        result[name.strip()] = float(raw_value)
    return result


def make_120m_plan(source: Path) -> Path:
    payload = json.loads(source.read_text(encoding="utf-8"))
    for item in payload["mission"]["items"]:
        if item.get("command") in (mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                   mavutil.mavlink.MAV_CMD_NAV_WAYPOINT):
            item["params"][6] = max(120.0, float(item["params"][6]))
    temporary = tempfile.NamedTemporaryFile(
        mode="w", suffix=".plan", prefix="honghu_pitch_id_", delete=False, encoding="utf-8"
    )
    with temporary:
        json.dump(payload, temporary, ensure_ascii=False, indent=2)
    return Path(temporary.name)


def send_gcs_heartbeat(connection) -> None:
    connection.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def send_rate_target(connection, target_system: int, target_component: int,
                     pitch_rate_deg_s: float, thrust: float) -> None:
    # Attitude is ignored; all body rates remain finite so the rate controller
    # retains roll/yaw stabilization while pitch is deliberately excited.
    connection.mav.set_attitude_target_send(
        int(time.monotonic() * 1000.0) & 0xFFFFFFFF,
        target_system,
        target_component,
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE,
        [1.0, 0.0, 0.0, 0.0],
        0.0,
        math.radians(pitch_rate_deg_s),
        0.0,
        thrust,
    )


def send_attitude_target(connection, target_system: int, target_component: int,
                         pitch_deg: float, yaw_rad: float, thrust: float) -> None:
    type_mask = (
        mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
        | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
        | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
    )


def speed_hold_thrust(state: dict, nominal_thrust: float,
                      target_airspeed_m_s: float, dt_s: float = 0.02) -> float:
    """Very slow throttle hold, kept below the 0.15 Hz pitch excitation band."""
    previous = float(state.get("_ident_thrust", nominal_thrust))
    measured = float(state["VFR_HUD"].airspeed)
    desired = max(0.2, min(1.0, nominal_thrust + 0.04 * (target_airspeed_m_s - measured)))
    alpha = max(0.0, min(1.0, dt_s / 3.0))
    value = previous + alpha * (desired - previous)
    state["_ident_thrust"] = value
    return value
    connection.mav.set_attitude_target_send(
        int(time.monotonic() * 1000.0) & 0xFFFFFFFF,
        target_system,
        target_component,
        type_mask,
        quaternion_from_euler(0.0, math.radians(pitch_deg), yaw_rad),
        0.0, 0.0, 0.0,
        thrust,
    )


def receive_state(connection, state: dict, timeout: float = 0.03) -> None:
    message = connection.recv_match(
        type=["ATTITUDE", "LOCAL_POSITION_NED", "VFR_HUD", "MISSION_CURRENT",
              "ATTITUDE_TARGET", "HEARTBEAT"],
        blocking=True,
        timeout=timeout,
    )
    if message is not None:
        state[message.get_type()] = message


def wait_for_cruise(dynamic_run: DynamicRun, initial_local, airspeed_m_s: float) -> dict:
    connection = dynamic_run.connection
    state: dict = {"LOCAL_POSITION_NED": initial_local}
    stable_since = None
    last_heartbeat = 0.0
    deadline = time.monotonic() + 420.0
    speed_command_sent = False
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now - last_heartbeat > 0.8:
            send_gcs_heartbeat(connection)
            last_heartbeat = now
        receive_state(connection, state, 0.1)
        if not all(name in state for name in (
            "ATTITUDE", "LOCAL_POSITION_NED", "VFR_HUD", "MISSION_CURRENT", "ATTITUDE_TARGET"
        )):
            continue
        attitude = state["ATTITUDE"]
        local = state["LOCAL_POSITION_NED"]
        vfr = state["VFR_HUD"]
        mission = state["MISSION_CURRENT"]
        altitude_gain = float(initial_local.z) - float(local.z)
        if int(mission.seq) >= 2 and altitude_gain >= 115.0 and not speed_command_sent:
            # Keep FW_AIRSPD_TRIM fixed at the production 45 m/s value so the
            # rate controller retains its real airspeed scaling.  Navigator's
            # cruise-speed command changes only the TECS operating point.
            connection.mav.command_long_send(
                dynamic_run.target_system,
                dynamic_run.target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0,
                0.0, airspeed_m_s, -1.0, 0.0, 0.0, 0.0, 0.0,
            )
            speed_command_sent = True
        ready = (
            int(mission.seq) >= 2
            and altitude_gain >= 115.0
            and abs(float(local.vz)) <= 1.0
            and abs(float(vfr.airspeed) - airspeed_m_s) <= 3.0
            and abs(math.degrees(float(attitude.pitch))) <= 6.0
        )
        if ready:
            stable_since = stable_since or now
            if now - stable_since >= 3.0:
                return state
        else:
            stable_since = None
    raise TimeoutError("aircraft did not reach the bounded 120 m identification condition")


def safety_check(state: dict, initial_z: float) -> None:
    attitude = state["ATTITUDE"]
    local = state["LOCAL_POSITION_NED"]
    vfr = state["VFR_HUD"]
    altitude_gain = initial_z - float(local.z)
    failures = []
    if abs(math.degrees(float(attitude.pitch))) > 10.0:
        failures.append("|pitch| > 10 deg")
    if abs(math.degrees(float(attitude.pitchspeed))) > 15.0:
        failures.append("|q| > 15 deg/s")
    if not 32.0 <= float(vfr.airspeed) <= 55.0:
        failures.append("TAS outside 32..55 m/s")
    if altitude_gain < 80.0:
        failures.append("AGL below 80 m")
    heartbeat = state.get("HEARTBEAT")
    if state.get("_expect_offboard") and heartbeat is not None:
        if mavutil.mode_string_v10(heartbeat).upper() != "OFFBOARD":
            failures.append("PX4 left OFFBOARD mode")
    if failures:
        raise RuntimeError("identification safety abort: " + ", ".join(failures))


def sample_state(elapsed_s: float, phase: str, command: float,
                 state: dict, initial_z: float) -> dict:
    attitude = state["ATTITUDE"]
    local = state["LOCAL_POSITION_NED"]
    vfr = state["VFR_HUD"]
    return {
        "elapsed_s": elapsed_s,
        "boot_time_s": float(attitude.time_boot_ms) * 1e-3,
        "phase": phase,
        "command": command,
        "roll_deg": math.degrees(float(attitude.roll)),
        "pitch_deg": math.degrees(float(attitude.pitch)),
        "pitch_rate_deg_s": math.degrees(float(attitude.pitchspeed)),
        "yaw_deg": math.degrees(float(attitude.yaw)),
        "airspeed_m_s": float(vfr.airspeed),
        "groundspeed_m_s": math.hypot(float(local.vx), float(local.vy)),
        "altitude_gain_m": initial_z - float(local.z),
        "vz_down_m_s": float(local.vz),
    }


def run_step_schedule(dynamic_run: DynamicRun, mode: str, state: dict,
                      initial_z: float, thrust: float,
                      trim_rate_deg_s: float, trim_pitch_deg: float,
                      target_airspeed_m_s: float,
                      samples: list[dict] | None = None) -> list[dict]:
    schedule = RATE_STEPS if mode == "rate" else ATTITUDE_STEPS
    samples = [] if samples is None else samples
    start = time.monotonic()
    phase_start = start
    phase_index = 0
    last_heartbeat = 0.0
    yaw_rad = float(state["ATTITUDE"].yaw)
    total_duration = sum(duration for _, duration in schedule)
    while time.monotonic() - start <= total_duration:
        now = time.monotonic()
        while phase_index + 1 < len(schedule) and now - phase_start >= schedule[phase_index][1]:
            phase_start += schedule[phase_index][1]
            phase_index += 1
        command = schedule[phase_index][0]
        if now - last_heartbeat > 0.8:
            send_gcs_heartbeat(dynamic_run.connection)
            last_heartbeat = now
        commanded_thrust = speed_hold_thrust(state, thrust, target_airspeed_m_s)
        if mode == "rate":
            send_rate_target(dynamic_run.connection, dynamic_run.target_system,
                             dynamic_run.target_component, trim_rate_deg_s + command, commanded_thrust)
        else:
            send_attitude_target(dynamic_run.connection, dynamic_run.target_system,
                                 dynamic_run.target_component, trim_pitch_deg + command, yaw_rad, commanded_thrust)
        receive_state(dynamic_run.connection, state)
        if all(name in state for name in ("ATTITUDE", "LOCAL_POSITION_NED", "VFR_HUD")):
            samples.append(sample_state(now - start, f"step_{phase_index}", command, state, initial_z))
            samples[-1]["absolute_command"] = (
                trim_rate_deg_s + command if mode == "rate" else trim_pitch_deg + command
            )
            safety_check(state, initial_z)
    return samples


def run_multisine(dynamic_run: DynamicRun, state: dict, initial_z: float,
                  thrust: float, trim_rate_deg_s: float,
                  target_airspeed_m_s: float,
                  duration_s: float = 40.0,
                  samples: list[dict] | None = None) -> list[dict]:
    samples = [] if samples is None else samples
    start = time.monotonic()
    last_heartbeat = 0.0
    phases = np.linspace(0.0, math.pi, len(MULTISINE_FREQUENCIES_HZ), endpoint=False)
    # Normalize the deterministic waveform to a strict +/-2.5 deg/s envelope.
    preview_time = np.linspace(0.0, duration_s, int(duration_s * 100), endpoint=False)
    preview = sum(
        np.sin(2.0 * math.pi * frequency * preview_time + phase)
        for frequency, phase in zip(MULTISINE_FREQUENCIES_HZ, phases)
    )
    scale = 2.5 / max(np.max(np.abs(preview)), 1.0)
    while time.monotonic() - start <= duration_s:
        now = time.monotonic()
        elapsed = now - start
        command = scale * sum(
            math.sin(2.0 * math.pi * frequency * elapsed + phase)
            for frequency, phase in zip(MULTISINE_FREQUENCIES_HZ, phases)
        )
        if now - last_heartbeat > 0.8:
            send_gcs_heartbeat(dynamic_run.connection)
            last_heartbeat = now
        commanded_thrust = speed_hold_thrust(state, thrust, target_airspeed_m_s)
        send_rate_target(dynamic_run.connection, dynamic_run.target_system,
                         dynamic_run.target_component, trim_rate_deg_s + command, commanded_thrust)
        receive_state(dynamic_run.connection, state)
        if all(name in state for name in ("ATTITUDE", "LOCAL_POSITION_NED", "VFR_HUD")):
            samples.append(sample_state(elapsed, "multisine", command, state, initial_z))
            samples[-1]["absolute_command"] = trim_rate_deg_s + command
            safety_check(state, initial_z)
    return samples


def prime_offboard(dynamic_run: DynamicRun, mode: str, state: dict, thrust: float,
                   trim_rate_deg_s: float, trim_pitch_deg: float,
                   initial_z: float, target_airspeed_m_s: float) -> None:
    start = time.monotonic()
    yaw_rad = float(state["ATTITUDE"].yaw)
    hold_pitch_deg = math.degrees(float(state["ATTITUDE"].pitch))
    hold_z = float(state["LOCAL_POSITION_NED"].z)
    state["_ident_thrust"] = thrust
    while time.monotonic() - start < 1.5:
        send_gcs_heartbeat(dynamic_run.connection)
        commanded_thrust = speed_hold_thrust(state, thrust, target_airspeed_m_s)
        if mode == "rate":
            # Enter OFFBOARD while holding the current attitude first.  A
            # direct AUTO q_sp -> zero-rate switch can integrate several
            # degrees of pitch before the rate integrator learns the trim,
            # especially at 50 m/s, and is not an aircraft instability.
            send_attitude_target(dynamic_run.connection, dynamic_run.target_system,
                                 dynamic_run.target_component, hold_pitch_deg, yaw_rad, commanded_thrust)
        else:
            send_attitude_target(dynamic_run.connection, dynamic_run.target_system,
                                 dynamic_run.target_component, trim_pitch_deg, yaw_rad, commanded_thrust)
        receive_state(dynamic_run.connection, state)
    dynamic_run.connection.set_mode_px4("OFFBOARD", 0, 0)
    commander_output = run(
        [str(PX4_BIN / "px4-commander"), "mode", "offboard"], check=False
    )
    mode_deadline = time.monotonic() + 6.0
    retry_time = time.monotonic() + 2.0
    while time.monotonic() < mode_deadline:
        send_gcs_heartbeat(dynamic_run.connection)
        commanded_thrust = speed_hold_thrust(state, thrust, target_airspeed_m_s)
        if mode == "rate":
            send_attitude_target(dynamic_run.connection, dynamic_run.target_system,
                                 dynamic_run.target_component, hold_pitch_deg, yaw_rad, commanded_thrust)
        else:
            send_attitude_target(dynamic_run.connection, dynamic_run.target_system,
                                 dynamic_run.target_component, trim_pitch_deg, yaw_rad, commanded_thrust)
        receive_state(dynamic_run.connection, state, 0.1)
        heartbeat = state.get("HEARTBEAT")
        if heartbeat is not None and mavutil.mode_string_v10(heartbeat).upper() == "OFFBOARD":
            state["_expect_offboard"] = True
            break
        if time.monotonic() >= retry_time:
            dynamic_run.connection.set_mode_px4("OFFBOARD", 0, 0)
            commander_output += run(
                [str(PX4_BIN / "px4-commander"), "mode", "offboard"], check=False
            )
            retry_time = float("inf")
    else:
        raise RuntimeError(
            "PX4 did not enter OFFBOARD mode; commander output: "
            + commander_output.strip()
        )
    if mode == "rate":
        hold_start = time.monotonic()
        hold_deadline = hold_start + 100.0
        stable_since = None
        while time.monotonic() < hold_deadline:
            send_gcs_heartbeat(dynamic_run.connection)
            current_z = float(state["LOCAL_POSITION_NED"].z)
            down_velocity = float(state["LOCAL_POSITION_NED"].vz)
            altitude_correction_deg = max(
                -3.0, min(3.0, 0.08 * (current_z - hold_z) + 0.6 * down_velocity)
            )
            commanded_thrust = speed_hold_thrust(state, thrust, target_airspeed_m_s)
            send_attitude_target(dynamic_run.connection, dynamic_run.target_system,
                                 dynamic_run.target_component,
                                 hold_pitch_deg + altitude_correction_deg,
                                 yaw_rad, commanded_thrust)
            receive_state(dynamic_run.connection, state)
            safety_check(state, initial_z)
            target = state.get("ATTITUDE_TARGET")
            actual_rate_deg_s = math.degrees(float(state["ATTITUDE"].pitchspeed))
            commanded_rate_deg_s = (
                math.degrees(float(target.body_pitch_rate)) if target is not None
                else float("inf")
            )
            converged = (
                time.monotonic() - hold_start >= 6.0
                and abs(actual_rate_deg_s) < 0.2
                and abs(commanded_rate_deg_s) < 0.25
            )
            if converged:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= 3.0:
                    break
            else:
                stable_since = None
        else:
            raise RuntimeError(
                "rate-loop trim preconditioning did not converge within 100 s"
            )
        rate_end = time.monotonic() + 2.0
        while time.monotonic() < rate_end:
            send_gcs_heartbeat(dynamic_run.connection)
            commanded_thrust = speed_hold_thrust(state, thrust, target_airspeed_m_s)
            send_rate_target(dynamic_run.connection, dynamic_run.target_system,
                             dynamic_run.target_component, trim_rate_deg_s, commanded_thrust)
            receive_state(dynamic_run.connection, state)
            safety_check(state, initial_z)


def attach_ulog_signals(samples: list[dict], ulog_path: Path) -> None:
    topics = ["honghu_v8_aero_state", "vehicle_torque_setpoint", "rate_ctrl_status"]
    ulog = ULog(str(ulog_path), message_name_filter_list=topics)
    fields = (
        ("elevator_deg", "honghu_v8_aero_state", "delta_doc_deg[1]"),
        ("pitch_torque_sp", "vehicle_torque_setpoint", "xyz[1]"),
        ("pitch_integrator", "rate_ctrl_status", "pitchspeed_integ"),
    )
    sample_time = np.asarray([sample["boot_time_s"] for sample in samples])
    for output_name, topic, field in fields:
        try:
            dataset = ulog.get_dataset(topic)
        except (IndexError, KeyError):
            # Custom diagnostics are optional logger topics.  The live rate
            # identification remains valid if a startup subscription race
            # excludes one of them; record NaN rather than discarding a whole
            # completed flight.
            for sample in samples:
                sample[output_name] = float("nan")
            continue
        timestamp = np.asarray(dataset.data["timestamp"], dtype=float) * 1e-6
        values = np.asarray(dataset.data[field], dtype=float)
        interpolated = np.interp(sample_time, timestamp, values)
        for sample, value in zip(samples, interpolated):
            sample[output_name] = float(value)


def step_metrics(samples: list[dict], mode: str) -> list[dict]:
    result = []
    output_name = "pitch_rate_deg_s" if mode == "rate" else "pitch_deg"
    schedule = RATE_STEPS if mode == "rate" else ATTITUDE_STEPS
    for index, (command, duration) in enumerate(schedule):
        phase = [sample for sample in samples if sample["phase"] == f"step_{index}"]
        if not phase:
            continue
        # Commands in ATTITUDE_STEPS are perturbations about the captured AUTO
        # pitch target, whereas pitch_deg is absolute.  Express both in the
        # same perturbation coordinates before evaluating tracking.  For rate
        # identification the equilibrium command is zero, so the same formula
        # remains valid.
        equilibrium_command = phase[0].get("absolute_command", command) - command
        response = [sample[output_name] - equilibrium_command for sample in phase]
        initial = response[0]
        final_window = [
            value for sample, value in zip(phase, response)
            if phase[-1]["elapsed_s"] - sample["elapsed_s"] <= 1.0
        ]
        final = statistics.fmean(final_window)
        change = command - initial
        direction = 1.0 if change >= 0.0 else -1.0
        directional_peak = max(direction * value for value in response) * direction
        overshoot = direction * (directional_peak - command)
        rise_90 = None
        threshold = initial + 0.9 * change
        for sample, value in zip(phase, response):
            if direction * (value - threshold) >= 0.0:
                rise_90 = sample["elapsed_s"] - phase[0]["elapsed_s"]
                break
        result.append({
            "phase": index,
            "command": command,
            "duration_s": duration,
            "initial": initial,
            "final_mean": final,
            "steady_error": command - final,
            "overshoot": max(0.0, overshoot),
            "rise_time_90_s": rise_90,
        })
    return result


def rate_frequency_metrics(samples: list[dict]) -> dict:
    multisine = [sample for sample in samples if sample["phase"] == "multisine"]
    if len(multisine) < 128:
        return {}
    time_s = np.asarray([sample["elapsed_s"] for sample in multisine])
    uniform_time = np.arange(time_s[0], time_s[-1], 0.02)
    command = np.interp(uniform_time, time_s, [sample["command"] for sample in multisine])
    response = np.interp(uniform_time, time_s, [sample["pitch_rate_deg_s"] for sample in multisine])
    spectrum = welch_cross_spectrum(command, response, 50.0, 1024)
    frequency_results = {
        f"{frequency:.2f}": transfer_at(spectrum, frequency)
        for frequency in MULTISINE_FREQUENCIES_HZ
    }
    valid_peak = [
        entry["gain"] for entry in frequency_results.values() if entry["coherence"] >= 0.8
    ]
    return {
        "frequencies": frequency_results,
        "peak_gain_with_coherence_ge_0_8": max(valid_peak, default=float("nan")),
        "target_0_234_hz": transfer_at(spectrum, 0.234),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("rate", "attitude"))
    parser.add_argument("--airspeed", type=float, choices=(40.0, 45.0, 50.0), default=45.0)
    parser.add_argument("--param", action="append", default=[], metavar="NAME=VALUE")
    parser.add_argument("--step-size", type=float, default=0.002)
    parser.add_argument("--mavlink-port", type=int, default=15550)
    parser.add_argument("--plan", type=Path, default=DEFAULT_PLAN)
    parser.add_argument("--output", type=Path)
    arguments = parser.parse_args()

    # Codex/CI sandboxes may expose the desktop ccache temporary directory as
    # read-only.  Keep this run self-contained without changing ccache size or
    # the user's persistent ccache configuration.
    ccache_temp = ROOT / "build/.ccache-tmp"
    ccache_temp.mkdir(parents=True, exist_ok=True)
    os.environ["CCACHE_TEMPDIR"] = str(ccache_temp)

    plan_path = make_120m_plan(arguments.plan)
    parameter_overrides = parse_parameter_overrides(arguments.param)
    dynamic_run = DynamicRun(
        scenario="standard",
        step_size=arguments.step_size,
        timeout_s=600.0,
        parameter_overrides=parameter_overrides,
        mavlink_port=arguments.mavlink_port,
        standard_plan=plan_path,
        make_target="gz_honghu_wing_100kg_v8_xiangyi_test",
        expected_canard_deg=6.0,
    )
    samples: list[dict] = []
    ulog_path = None
    abort_reason = None
    trim_rate_deg_s = None
    trim_pitch_deg = None
    try:
        dynamic_run.start()
        _, initial_local = dynamic_run.wait_for_position()
        dynamic_run.upload_standard_plan()
        dynamic_run.arm_and_start_mission()
        state = wait_for_cruise(dynamic_run, initial_local, arguments.airspeed)
        target = state.get("ATTITUDE_TARGET")
        thrust = float(target.thrust) if target is not None and math.isfinite(float(target.thrust)) \
            else max(0.2, min(1.0, float(state["VFR_HUD"].throttle) / 100.0))
        pre_switch_rate_command_deg_s = math.degrees(float(target.body_pitch_rate))
        # Identification is performed about a true zero-rate equilibrium.
        # A non-zero AUTO rate setpoint is evidence of attitude-error trim and
        # must not be mistaken for an equilibrium body rate.
        trim_rate_deg_s = 0.0
        trim_pitch_deg = math.degrees(pitch_from_quaternion(target.q))
        prime_offboard(
            dynamic_run, arguments.mode, state, thrust, trim_rate_deg_s, trim_pitch_deg,
            float(initial_local.z), arguments.airspeed
        )
        run_step_schedule(
            dynamic_run, arguments.mode, state, float(initial_local.z), thrust,
            trim_rate_deg_s, trim_pitch_deg, arguments.airspeed, samples
        )
        if arguments.mode == "rate":
            run_multisine(
                dynamic_run, state, float(initial_local.z), thrust, trim_rate_deg_s,
                arguments.airspeed, samples=samples
            )
        run([str(PX4_BIN / "px4-commander"), "mode", "auto:loiter"], check=False)
        recovery_end = time.monotonic() + 2.0
        while time.monotonic() < recovery_end:
            send_gcs_heartbeat(dynamic_run.connection)
            receive_state(dynamic_run.connection, state, 0.05)
    except Exception as error:
        abort_reason = str(error)
        try:
            run([str(PX4_BIN / "px4-commander"), "mode", "auto:loiter"], check=False)
        except Exception:
            pass
    finally:
        dynamic_run.stop()
        plan_path.unlink(missing_ok=True)

    new_logs = sorted(
        set(ROOTFS.glob("log/**/*.ulg")) - dynamic_run.ulog_files_before,
        key=lambda path: path.stat().st_mtime,
    )
    if new_logs:
        ulog_path = new_logs[-1]
        if samples:
            attach_ulog_signals(samples, ulog_path)

    report = {
        "status": "PASS" if samples and abort_reason is None else "FAIL",
        "mode": arguments.mode,
        "airspeed_target_m_s": arguments.airspeed,
        "parameter_overrides": parameter_overrides,
        "trim_rate_command_deg_s": trim_rate_deg_s,
        "pre_switch_rate_command_deg_s": locals().get("pre_switch_rate_command_deg_s"),
        "trim_pitch_command_deg": trim_pitch_deg,
        "abort_reason": abort_reason,
        "ulog": str(ulog_path) if ulog_path else None,
        "step_metrics": step_metrics(samples, arguments.mode),
        "frequency_metrics": rate_frequency_metrics(samples) if arguments.mode == "rate" else {},
        "safety_extrema": {
            "pitch_max_abs_deg": max((abs(sample["pitch_deg"]) for sample in samples), default=float("nan")),
            "pitch_rate_max_abs_deg_s": max((abs(sample["pitch_rate_deg_s"]) for sample in samples), default=float("nan")),
            "airspeed_min_m_s": min((sample["airspeed_m_s"] for sample in samples), default=float("nan")),
            "airspeed_max_m_s": max((sample["airspeed_m_s"] for sample in samples), default=float("nan")),
            "altitude_gain_min_m": min((sample["altitude_gain_m"] for sample in samples), default=float("nan")),
            "elevator_max_abs_deg": max((abs(sample.get("elevator_deg", float("nan"))) for sample in samples if math.isfinite(sample.get("elevator_deg", float("nan")))), default=float("nan")),
            "pitch_integrator_max_abs": max((abs(sample.get("pitch_integrator", float("nan"))) for sample in samples if math.isfinite(sample.get("pitch_integrator", float("nan")))), default=float("nan")),
        },
        "sample_count": len(samples),
        "samples": samples,
    }
    output = arguments.output or Path(
        "analysis_outputs/honghu_v8_100kg_pitch_identification/"
        f"{arguments.mode}_{arguments.airspeed:g}mps.json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    # Keep unattended sweeps readable.  The complete step-by-step metrics and
    # every sample remain in the JSON artifact.
    console_report = {
        key: value for key, value in report.items()
        if key not in ("samples", "step_metrics")
    }
    print(json.dumps(console_report, ensure_ascii=False, indent=2))
    if report["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
