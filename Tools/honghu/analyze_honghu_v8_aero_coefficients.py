#!/usr/bin/env python3
"""Reconstruct and compare Honghu V8 six-component aerodynamic coefficients.

Two independent paths are evaluated on a common ULog time base:

1. Rigid-body inversion reconstructs physical body specific force from PX4's
   filtered ``vehicle_acceleration`` plus the EKF acceleration-bias estimate,
   then uses angular velocity / angular acceleration, the full inertia tensor,
   and a separately evaluated propulsion model.  Adding the bias back is
   essential: ``vehicle_acceleration`` has already had that estimate removed
   and is therefore a controller signal, not an untouched force-balance input.
2. The V8 forward model uses reconstructed alpha / beta, body rates, actual
   Gazebo joint feedback, and the CSV aerodynamic tables.

Ground-contact and low-dynamic-pressure samples are excluded.  The script does
not identify or fit any aerodynamic parameter.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from dataclasses import asdict
from pathlib import Path
from typing import Dict, Iterable, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pyulog import ULog

from honghu_v8_aero_model import HonghuV8AeroModel, isa_density
from honghu_v8_propulsion_model import HonghuV8PropulsionModel


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PLAN = Path("/home/fly/px4_reference_docs/current/模仿XY航线规划.plan")
COEFFICIENTS = ("CL", "CD", "CY", "Cl", "Cm", "Cn")
MASS_KG = 150.0
AREA_M2 = 2.42
SPAN_M = 3.96
MAC_M = 0.62
GROUND_BASE_Z_M = 0.5145
GAZEBO_GRAVITY_MPS2 = 9.8
THRUST_DOWN_RAD = math.radians(3.0)
ENGINE_POINT_FRD_M = np.array([-1.23, 0.0, -0.12])

# The SDF tensor is expressed in Gazebo FLU. Transforming both tensor indices
# with diag(1,-1,-1) yields this PX4/PDF FRD tensor.
INERTIA_FRD_KGM2 = np.array(
    [
        [25.86, -0.017, -3.520],
        [-0.017, 39.14, -0.0019],
        [-3.520, -0.0019, 59.12],
    ]
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def get_dataset(ulog: ULog, name: str, multi_id: int = 0):
    for dataset in ulog.data_list:
        if dataset.name == name and dataset.multi_id == multi_id:
            return dataset
    raise KeyError(f"ULog is missing {name}[{multi_id}]")


def optional_dataset(ulog: ULog, name: str, multi_id: int = 0):
    for dataset in ulog.data_list:
        if dataset.name == name and dataset.multi_id == multi_id:
            return dataset
    return None


def sample_time(dataset) -> np.ndarray:
    data = dataset.data
    if "timestamp_sample" in data:
        candidate = np.asarray(data["timestamp_sample"], dtype=float)
        if len(candidate) and np.count_nonzero(candidate > 0) > len(candidate) // 2:
            return candidate * 1e-6
    return np.asarray(data["timestamp"], dtype=float) * 1e-6


def continuous(dataset, fields: Sequence[str], target_time: np.ndarray) -> np.ndarray:
    source_time = sample_time(dataset)
    order = np.argsort(source_time)
    source_time = source_time[order]
    return np.column_stack(
        [
            np.interp(target_time, source_time, np.asarray(dataset.data[field], dtype=float)[order])
            for field in fields
        ]
    )


def previous_value(dataset, field: str, target_time: np.ndarray) -> np.ndarray:
    source_time = sample_time(dataset)
    order = np.argsort(source_time)
    source_time = source_time[order]
    values = np.asarray(dataset.data[field], dtype=float)[order]
    indices = np.searchsorted(source_time, target_time, side="right") - 1
    indices = np.clip(indices, 0, len(source_time) - 1)
    return values[indices]


def normalize_quaternions(quaternions: np.ndarray) -> np.ndarray:
    result = quaternions.copy()
    for index in range(1, len(result)):
        if np.dot(result[index - 1], result[index]) < 0.0:
            result[index] *= -1.0
    norms = np.linalg.norm(result, axis=1)
    result /= np.maximum(norms[:, None], 1e-12)
    return result


def quaternion_body_to_world(quaternions: np.ndarray) -> np.ndarray:
    """Return FRD-body to NED-world DCMs for scalar-first PX4 quaternions."""
    q = normalize_quaternions(quaternions)
    w, x, y, z = q.T
    matrices = np.empty((len(q), 3, 3))
    matrices[:, 0, 0] = 1.0 - 2.0 * (y * y + z * z)
    matrices[:, 0, 1] = 2.0 * (x * y - w * z)
    matrices[:, 0, 2] = 2.0 * (x * z + w * y)
    matrices[:, 1, 0] = 2.0 * (x * y + w * z)
    matrices[:, 1, 1] = 1.0 - 2.0 * (x * x + z * z)
    matrices[:, 1, 2] = 2.0 * (y * z - w * x)
    matrices[:, 2, 0] = 2.0 * (x * z - w * y)
    matrices[:, 2, 1] = 2.0 * (y * z + w * x)
    matrices[:, 2, 2] = 1.0 - 2.0 * (x * x + y * y)
    return matrices


def plugin_angle_rates(
    time_s: np.ndarray, alpha_rad: np.ndarray, beta_rad: np.ndarray, speed_mps: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Approximate HonghuAeroV8's clamped 50 ms alpha/beta derivative filter."""
    alpha_dot = np.zeros_like(alpha_rad)
    beta_dot = np.zeros_like(beta_rad)
    previous_alpha = alpha_rad[0]
    previous_beta = beta_rad[0]
    filtered_alpha = 0.0
    filtered_beta = 0.0
    have_angles = False
    for index in range(len(time_s)):
        dt = time_s[index] - time_s[index - 1] if index else 0.0
        if speed_mps[index] < 3.0 or dt <= 0.0 or not have_angles:
            previous_alpha = alpha_rad[index]
            previous_beta = beta_rad[index]
            filtered_alpha = 0.0
            filtered_beta = 0.0
            have_angles = True
        else:
            da = math.remainder(alpha_rad[index] - previous_alpha, 2.0 * math.pi)
            db = math.remainder(beta_rad[index] - previous_beta, 2.0 * math.pi)
            raw_alpha = float(np.clip(da / dt, -10.0, 10.0))
            raw_beta = float(np.clip(db / dt, -10.0, 10.0))
            gain = dt / (0.05 + dt)
            filtered_alpha += gain * (raw_alpha - filtered_alpha)
            filtered_beta += gain * (raw_beta - filtered_beta)
            previous_alpha = alpha_rad[index]
            previous_beta = beta_rad[index]
        alpha_dot[index] = filtered_alpha
        beta_dot[index] = filtered_beta
    return alpha_dot, beta_dot


def engine_state(
    target_time: np.ndarray,
    target_command: np.ndarray,
    output_time: np.ndarray,
    step_s: float = 0.002,
) -> np.ndarray:
    """Reproduce the V8 0.5/0.3 s asymmetric first-order engine state."""
    start = 0.0
    grid = np.arange(start, float(output_time[-1]) + step_s, step_s)
    target = np.interp(grid, target_time, target_command, left=0.0, right=target_command[-1])
    state = np.zeros_like(grid)
    for index in range(1, len(grid)):
        tau = 0.5 if target[index] > state[index - 1] else 0.3
        gain = min(step_s / max(tau, 1e-3), 1.0)
        state[index] = state[index - 1] + gain * (target[index] - state[index - 1])
    return np.interp(output_time, grid, state)


def surface_angles_from_outputs(outputs: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    normalized = np.clip(2.0 * outputs / 1000.0 - 1.0, -1.0, 1.0)
    theta_deg = np.zeros_like(normalized)
    theta_deg[:, :6] = 30.0 * normalized[:, :6]
    theta_deg[:, 6:8] = np.where(
        normalized[:, 6:8] <= 0.0,
        50.0 * normalized[:, 6:8],
        15.0 * normalized[:, 6:8],
    )
    return theta_deg, document_deflections_from_joint_angles(theta_deg)


def document_deflections_from_joint_angles(theta_deg: np.ndarray) -> np.ndarray:
    """Apply the V8 SDF-joint to PDF-control sign contract."""
    return np.column_stack(
        (
            0.5 * (-theta_deg[:, 0] + theta_deg[:, 1]),
            0.5 * (theta_deg[:, 2] + theta_deg[:, 3]),
            0.5 * (theta_deg[:, 4] + theta_deg[:, 5]),
            0.5 * (theta_deg[:, 6] + theta_deg[:, 7]),
        )
    )


def diagnostic_timing(dataset) -> Dict[str, object]:
    arrival_us = np.asarray(dataset.data["timestamp"], dtype=np.int64)
    source_us = np.asarray(dataset.data.get("timestamp_sample", arrival_us), dtype=np.int64)
    latency_ms = (arrival_us - source_us) * 1e-3
    result: Dict[str, object] = {
        "samples": int(len(arrival_us)),
        "source_timestamp_strictly_monotonic": bool(
            len(source_us) < 2 or np.all(np.diff(source_us) > 0)
        ),
        "arrival_minus_source_latency_ms_median": float(np.median(latency_ms)),
        "arrival_minus_source_latency_ms_p95": float(np.percentile(latency_ms, 95)),
        "arrival_minus_source_latency_ms_max": float(np.max(latency_ms)),
    }
    if "sequence" in dataset.data:
        sequence = np.asarray(dataset.data["sequence"], dtype=np.int64)
        sequence_steps = np.diff(sequence)
        result.update(
            {
                "sequence_start": int(sequence[0]),
                "sequence_end": int(sequence[-1]),
                "sequence_strictly_monotonic": bool(
                    len(sequence) < 2 or np.all(sequence_steps > 0)
                ),
                "sequence_gap_count": int(np.count_nonzero(sequence_steps > 1)),
            }
        )
    return result


def nan_smooth(values: np.ndarray, window_samples: int) -> np.ndarray:
    if window_samples <= 1:
        return values.copy()
    if window_samples % 2 == 0:
        window_samples += 1
    kernel = np.hanning(window_samples)
    if not np.any(kernel):
        kernel = np.ones(window_samples)
    kernel /= kernel.sum()
    finite = np.isfinite(values)
    numerator = np.convolve(np.where(finite, values, 0.0), kernel, mode="same")
    denominator = np.convolve(finite.astype(float), kernel, mode="same")
    result = np.full_like(values, np.nan, dtype=float)
    usable = denominator > 0.5
    result[usable] = numerator[usable] / denominator[usable]
    return result


def metrics(measured: np.ndarray, predicted: np.ndarray, mask: np.ndarray) -> Dict[str, float]:
    valid = mask & np.isfinite(measured) & np.isfinite(predicted)
    x = measured[valid]
    y = predicted[valid]
    residual = x - y
    correlation = float(np.corrcoef(x, y)[0, 1]) if len(x) > 2 and np.std(x) > 0 and np.std(y) > 0 else math.nan
    return {
        "samples": int(len(x)),
        "bias_first_minus_second": float(np.mean(residual)),
        "mae": float(np.mean(np.abs(residual))),
        "rmse": float(np.sqrt(np.mean(residual**2))),
        "correlation": correlation,
        "inverse_mean": float(np.mean(x)),
        "model_mean": float(np.mean(y)),
        "inverse_std": float(np.std(x)),
        "model_std": float(np.std(y)),
        "p95_abs_error": float(np.percentile(np.abs(residual), 95)),
    }


def plot_coefficients(frame: pd.DataFrame, output: Path) -> None:
    valid = frame[frame["analysis_valid"] > 0.5]
    fig, axes = plt.subplots(3, 2, figsize=(15, 11), sharex=True)
    for axis, name in zip(axes.flat, COEFFICIENTS):
        axis.plot(valid["time_s"], valid[f"{name}_inverse_filtered"], color="#0072B2", lw=1.1, label="rigid-body inversion")
        axis.plot(valid["time_s"], valid[f"{name}_model_filtered"], color="#D55E00", lw=1.0, label="V8 forward model")
        truth_column = f"{name}_plugin_raw"
        if truth_column in valid and np.any(np.isfinite(valid[truth_column])):
            axis.plot(valid["time_s"], valid[truth_column], color="#009E73", lw=0.8, alpha=0.8, label="plugin diagnostic")
        axis.set_ylabel(name)
        axis.grid(True, alpha=0.3)
    axes[0, 0].legend(loc="best")
    axes[-1, 0].set_xlabel("simulation time [s]")
    axes[-1, 1].set_xlabel("simulation time [s]")
    fig.suptitle("Honghu V8 aerodynamic coefficients: dynamics inversion vs forward model")
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def plot_states(frame: pd.DataFrame, output: Path) -> None:
    valid = frame[frame["analysis_valid"] > 0.5]
    t = valid["time_s"]
    fig, axes = plt.subplots(4, 2, figsize=(15, 12), sharex=True)
    axes[0, 0].plot(t, valid["airspeed_mps"]); axes[0, 0].set_ylabel("V [m/s]")
    axes[0, 1].plot(t, valid["alpha_deg"], label="alpha"); axes[0, 1].plot(t, valid["beta_deg"], label="beta"); axes[0, 1].set_ylabel("angle [deg]"); axes[0, 1].legend()
    axes[1, 0].plot(t, valid["p_rad_s"], label="p"); axes[1, 0].plot(t, valid["q_rad_s"], label="q"); axes[1, 0].plot(t, valid["r_rad_s"], label="r"); axes[1, 0].set_ylabel("body rate [rad/s]"); axes[1, 0].legend()
    axes[1, 1].plot(t, valid["pdot_rad_s2"], label="pdot"); axes[1, 1].plot(t, valid["qdot_rad_s2"], label="qdot"); axes[1, 1].plot(t, valid["rdot_rad_s2"], label="rdot"); axes[1, 1].set_ylabel("angular accel [rad/s2]"); axes[1, 1].legend()
    axes[2, 0].plot(t, valid["delta_a_doc_deg"], label="aileron"); axes[2, 0].plot(t, valid["delta_e_doc_deg"], label="elevator"); axes[2, 0].plot(t, valid["delta_r_doc_deg"], label="rudder"); axes[2, 0].plot(t, valid["delta_c_doc_deg"], label="canard"); axes[2, 0].set_ylabel("delta_doc [deg]"); axes[2, 0].legend(ncol=2)
    axes[2, 1].plot(t, valid["throttle_target"], label="target"); axes[2, 1].plot(t, valid["throttle_state"], label="engine state"); axes[2, 1].set_ylabel("throttle"); axes[2, 1].legend()
    axes[3, 0].plot(t, valid["thrust_n"]); axes[3, 0].set_ylabel("thrust [N]"); axes[3, 0].set_xlabel("simulation time [s]")
    axes[3, 1].plot(t, valid["altitude_agl_m"]); axes[3, 1].set_ylabel("height AGL [m]"); axes[3, 1].set_xlabel("simulation time [s]")
    for axis in axes.flat:
        axis.grid(True, alpha=0.3)
    fig.suptitle("Honghu V8 coefficient-reconstruction inputs")
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def analyze(arguments: argparse.Namespace) -> Dict[str, object]:
    required_topics = [
        "vehicle_acceleration",
        "estimator_sensor_bias",
        "vehicle_angular_velocity",
        "vehicle_local_position_groundtruth",
        "vehicle_attitude_groundtruth",
        "vehicle_land_detected",
        "vehicle_air_data",
        "actuator_outputs",
        "mission_result",
        "honghu_v8_aero_state",
        "honghu_v8_propulsion_state",
    ]
    ulog = ULog(str(arguments.ulog), message_name_filter_list=required_topics)
    parameter_names = (
        "SYS_AUTOSTART", "SIM_GZ_SV_ZMAP",
        "FW_AIRSPD_MIN", "FW_AIRSPD_TRIM", "FW_AIRSPD_MAX",
        "FW_CANARD_NEUT", "FW_CANARD_TO", "FW_CANARD_BRK",
        "NPFG_PERIOD", "NAV_ACC_RAD", "FW_R_LIM",
    )
    accel_data = get_dataset(ulog, "vehicle_acceleration")
    accel_bias_data = optional_dataset(ulog, "estimator_sensor_bias")
    angular_data = get_dataset(ulog, "vehicle_angular_velocity")
    lpos_data = get_dataset(ulog, "vehicle_local_position_groundtruth")
    attitude_data = get_dataset(ulog, "vehicle_attitude_groundtruth")
    land_data = get_dataset(ulog, "vehicle_land_detected")
    air_data = get_dataset(ulog, "vehicle_air_data")
    motor_output = get_dataset(ulog, "actuator_outputs", 0)
    servo_output = get_dataset(ulog, "actuator_outputs", 1)
    mission_data = get_dataset(ulog, "mission_result")
    aero_diagnostic = optional_dataset(ulog, "honghu_v8_aero_state")
    propulsion_diagnostic = optional_dataset(ulog, "honghu_v8_propulsion_state")
    if aero_diagnostic is None and not getattr(arguments, "allow_commanded_surface_fallback", False):
        raise RuntimeError(
            "ULog has no honghu_v8_aero_state actual Gazebo joint feedback; "
            "rerun V8 after rebuilding, or explicitly pass "
            "--allow-commanded-surface-fallback for a historical log"
        )
    if accel_bias_data is None and not getattr(arguments, "allow_estimator_biased_acceleration", False):
        raise RuntimeError(
            "ULog has no estimator_sensor_bias, so the EKF bias removed from "
            "vehicle_acceleration cannot be restored; rerun with full logging, "
            "or explicitly pass --allow-estimator-biased-acceleration for a "
            "historical qualitative comparison"
        )

    t_accel = sample_time(accel_data)
    common_datasets = [angular_data, lpos_data, attitude_data, motor_output, servo_output]
    if accel_bias_data is not None:
        common_datasets.append(accel_bias_data)
    if aero_diagnostic is not None:
        common_datasets.append(aero_diagnostic)
    if propulsion_diagnostic is not None:
        common_datasets.append(propulsion_diagnostic)
    start = max([t_accel[0]] + [sample_time(dataset)[0] for dataset in common_datasets])
    end = min([t_accel[-1]] + [sample_time(dataset)[-1] for dataset in common_datasets])
    if arguments.start is not None:
        start = max(start, arguments.start)
    if arguments.end is not None:
        end = min(end, arguments.end)
    time_s = t_accel[(t_accel >= start) & (t_accel <= end)]
    if len(time_s) < 20:
        raise RuntimeError("insufficient common ULog time interval")

    vehicle_acceleration_frd = continuous(accel_data, ("xyz[0]", "xyz[1]", "xyz[2]"), time_s)
    if accel_bias_data is not None:
        # VehicleAcceleration::Run publishes low-pass filtered
        # (calibrated_acceleration - EKF_accel_bias). The aerodynamic force
        # balance needs the physical specific force, so restore that bias.
        accel_bias_frd = np.column_stack(
            [
                previous_value(accel_bias_data, f"accel_bias[{axis}]", time_s)
                for axis in range(3)
            ]
        )
        accel_bias_valid = previous_value(accel_bias_data, "accel_bias_valid", time_s) > 0.5
        acceleration_frd = vehicle_acceleration_frd + accel_bias_frd
        acceleration_source = (
            "vehicle_acceleration plus estimator_sensor_bias restored offline"
        )
    else:
        accel_bias_frd = np.zeros_like(vehicle_acceleration_frd)
        accel_bias_valid = np.ones(len(time_s), dtype=bool)
        acceleration_frd = vehicle_acceleration_frd
        acceleration_source = (
            "vehicle_acceleration with EKF acceleration bias still removed "
            "(explicit historical fallback)"
        )
    angular = continuous(
        angular_data,
        (
            "xyz[0]", "xyz[1]", "xyz[2]",
            "xyz_derivative[0]", "xyz_derivative[1]", "xyz_derivative[2]",
        ),
        time_s,
    )
    omega_frd = angular[:, :3]
    omega_dot_frd = angular[:, 3:]

    velocity_ned = continuous(lpos_data, ("vx", "vy", "vz"), time_s)
    position_ned = continuous(lpos_data, ("x", "y", "z"), time_s)
    quaternions = continuous(attitude_data, ("q[0]", "q[1]", "q[2]", "q[3]"), time_s)
    body_to_ned = quaternion_body_to_world(quaternions)
    groundtruth_acceleration_ned = continuous(lpos_data, ("ax", "ay", "az"), time_s)
    groundtruth_specific_force_frd = np.einsum(
        "nji,nj->ni",
        body_to_ned,
        groundtruth_acceleration_ned - np.array([0.0, 0.0, GAZEBO_GRAVITY_MPS2]),
    )
    velocity_frd = np.einsum("nji,nj->ni", body_to_ned, velocity_ned)
    airspeed = np.linalg.norm(velocity_frd, axis=1)
    alpha_rad = np.arctan2(velocity_frd[:, 2], velocity_frd[:, 0])
    beta_rad = np.arctan2(velocity_frd[:, 1], np.hypot(velocity_frd[:, 0], velocity_frd[:, 2]))

    # Compute the angle-rate state on the higher-rate pose velocity time base,
    # then sample it at the force inversion time stamps.
    high_time = sample_time(lpos_data)
    high_velocity_ned = continuous(lpos_data, ("vx", "vy", "vz"), high_time)
    high_quaternion = continuous(attitude_data, ("q[0]", "q[1]", "q[2]", "q[3]"), high_time)
    high_dcm = quaternion_body_to_world(high_quaternion)
    high_velocity_frd = np.einsum("nji,nj->ni", high_dcm, high_velocity_ned)
    high_speed = np.linalg.norm(high_velocity_frd, axis=1)
    high_alpha = np.arctan2(high_velocity_frd[:, 2], high_velocity_frd[:, 0])
    high_beta = np.arctan2(high_velocity_frd[:, 1], np.hypot(high_velocity_frd[:, 0], high_velocity_frd[:, 2]))
    high_alpha_dot, high_beta_dot = plugin_angle_rates(high_time, high_alpha, high_beta, high_speed)
    alpha_dot = np.interp(time_s, high_time, high_alpha_dot)
    beta_dot = np.interp(time_s, high_time, high_beta_dot)

    servo_fields = tuple(f"output[{index}]" for index in range(8))
    servo_outputs = continuous(servo_output, servo_fields, time_s)
    theta_deg, delta_doc = surface_angles_from_outputs(servo_outputs)
    surface_source = "actuator_outputs[1] command-angle reconstruction"
    plugin_coefficients = np.full((len(time_s), len(COEFFICIENTS)), np.nan)
    plugin_reported_delta = np.full((len(time_s), 4), np.nan)
    aero_flags = np.zeros(len(time_s), dtype=np.uint32)
    forward_omega_frd = omega_frd

    if aero_diagnostic is not None:
        diagnostic = continuous(
            aero_diagnostic,
            (
                "airspeed_m_s", "alpha_deg", "beta_deg", "rho_kg_m3",
                "alpha_dot_rad_s", "beta_dot_rad_s",
                "body_rates_frd_rad_s[0]", "body_rates_frd_rad_s[1]", "body_rates_frd_rad_s[2]",
                "coefficients[0]", "coefficients[1]", "coefficients[2]",
                "coefficients[3]", "coefficients[4]", "coefficients[5]",
                "joint_angles_deg[0]", "joint_angles_deg[1]", "joint_angles_deg[2]", "joint_angles_deg[3]",
                "joint_angles_deg[4]", "joint_angles_deg[5]", "joint_angles_deg[6]", "joint_angles_deg[7]",
                "delta_doc_deg[0]", "delta_doc_deg[1]", "delta_doc_deg[2]", "delta_doc_deg[3]",
            ),
            time_s,
        )
        # Keep kinematic state independent: airspeed, alpha/beta, rates and
        # density continue to come from ULog ground truth / the selected
        # atmosphere source. Only the physical Gazebo joint feedback is used as
        # the forward-model control input. Plugin coefficients remain a third,
        # diagnostic-only comparison trace.
        diagnostic_rho = diagnostic[:, 3]
        plugin_coefficients = diagnostic[:, 9:15]
        theta_deg = diagnostic[:, 15:23]
        plugin_reported_delta = diagnostic[:, 23:27]
        delta_doc = document_deflections_from_joint_angles(theta_deg)
        aero_flags = previous_value(aero_diagnostic, "flags", time_s).astype(np.uint32)
        surface_source = (
            "honghu_v8_aero_state actual Gazebo joint feedback; "
            "delta_doc independently recomputed from theta_joint"
        )
    else:
        diagnostic_rho = np.full(len(time_s), np.nan)

    motor_command = continuous(motor_output, ("output[0]",), time_s)[:, 0] / 1000.0
    motor_time = sample_time(motor_output)
    motor_target_source = np.asarray(motor_output.data["output[0]"], dtype=float) / 1000.0
    throttle_state = engine_state(motor_time, motor_target_source, time_s)

    altitude_m = -position_ned[:, 2]
    altitude_agl_m = altitude_m - GROUND_BASE_Z_M
    logged_rho = continuous(air_data, ("rho",), time_s)[:, 0]
    model_rho = np.asarray([isa_density(value) for value in altitude_m])
    rho = logged_rho if arguments.density_source == "logged" else model_rho
    qbar = 0.5 * rho * airspeed**2

    propulsion_model = HonghuV8PropulsionModel()
    propulsion_altitude = altitude_m
    propulsion_airspeed = airspeed
    plugin_propulsion = np.full((len(time_s), 8), np.nan)
    propulsion_flags = np.zeros(len(time_s), dtype=np.uint32)
    propulsion_source = "actuator_outputs[0] plus independently reconstructed 0.5/0.3 s lag state"

    if propulsion_diagnostic is not None:
        plugin_propulsion = continuous(
            propulsion_diagnostic,
            (
                "target_throttle", "filtered_throttle", "altitude_m", "airspeed_m_s",
                "rpm", "thrust_n", "torque_nm", "fuel_rate",
            ),
            time_s,
        )
        throttle_state = plugin_propulsion[:, 1]
        propulsion_altitude = plugin_propulsion[:, 2]
        propulsion_airspeed = plugin_propulsion[:, 3]
        propulsion_flags = previous_value(propulsion_diagnostic, "flags", time_s).astype(np.uint32)
        propulsion_source = "honghu_v8_propulsion_state lag state plus independent table evaluation"

    propulsion = propulsion_model.evaluate_many(propulsion_altitude, throttle_state, propulsion_airspeed)
    rpm = np.asarray([item.rpm for item in propulsion])
    thrust = np.asarray([item.thrust_newton for item in propulsion])
    engine_torque = np.asarray([item.torque_nm for item in propulsion])
    prop_clamped = np.asarray([item.clamped for item in propulsion], dtype=bool)
    prop_force_frd = np.column_stack(
        (thrust * math.cos(THRUST_DOWN_RAD), np.zeros_like(thrust), thrust * math.sin(THRUST_DOWN_RAD))
    )
    prop_reaction_frd = np.column_stack((-engine_torque, np.zeros_like(thrust), np.zeros_like(thrust)))
    prop_moment_frd = np.cross(np.broadcast_to(ENGINE_POINT_FRD_M, prop_force_frd.shape), prop_force_frd) + prop_reaction_frd

    aero_force_frd = MASS_KG * acceleration_frd - prop_force_frd
    inertia_omega = omega_frd @ INERTIA_FRD_KGM2.T
    total_moment_frd = omega_dot_frd @ INERTIA_FRD_KGM2.T + np.cross(omega_frd, inertia_omega)
    aero_moment_frd = total_moment_frd - prop_moment_frd

    ca, sa = np.cos(alpha_rad), np.sin(alpha_rad)
    cb, sb = np.cos(beta_rad), np.sin(beta_rad)
    ex = np.column_stack((ca * cb, sb, sa * cb))
    ey = np.column_stack((-ca * sb, cb, -sa * sb))
    ez = np.column_stack((-sa, np.zeros_like(sa), ca))
    denominator_force = qbar * AREA_M2
    force_denominator_safe = denominator_force > 1e-9
    roll_yaw_denominator = denominator_force * SPAN_M
    pitch_denominator = denominator_force * MAC_M
    force_x = np.einsum("ij,ij->i", aero_force_frd, ex)
    force_y = np.einsum("ij,ij->i", aero_force_frd, ey)
    force_z = np.einsum("ij,ij->i", aero_force_frd, ez)
    inverse = {
        "CD": np.divide(-force_x, denominator_force, out=np.full_like(force_x, np.nan), where=force_denominator_safe),
        "CY": np.divide(force_y, denominator_force, out=np.full_like(force_y, np.nan), where=force_denominator_safe),
        "CL": np.divide(-force_z, denominator_force, out=np.full_like(force_z, np.nan), where=force_denominator_safe),
        "Cl": np.divide(aero_moment_frd[:, 0], roll_yaw_denominator, out=np.full_like(force_x, np.nan), where=force_denominator_safe),
        "Cm": np.divide(aero_moment_frd[:, 1], pitch_denominator, out=np.full_like(force_x, np.nan), where=force_denominator_safe),
        "Cn": np.divide(aero_moment_frd[:, 2], roll_yaw_denominator, out=np.full_like(force_x, np.nan), where=force_denominator_safe),
    }

    aero_model = HonghuV8AeroModel()
    predicted_lists = {name: [] for name in COEFFICIENTS}
    for index in range(len(time_s)):
        coefficients, _ = aero_model.coefficients(
            math.degrees(alpha_rad[index]),
            math.degrees(beta_rad[index]),
            airspeed[index],
            delta_a_deg=delta_doc[index, 0],
            delta_e_deg=delta_doc[index, 1],
            delta_r_deg=delta_doc[index, 2],
            delta_c_deg=delta_doc[index, 3],
            p_rad_s=forward_omega_frd[index, 0],
            q_rad_s=forward_omega_frd[index, 1],
            r_rad_s=forward_omega_frd[index, 2],
            alpha_dot_rad_s=alpha_dot[index],
            beta_dot_rad_s=beta_dot[index],
        )
        for name in COEFFICIENTS:
            predicted_lists[name].append(getattr(coefficients, name))
    predicted = {name: np.asarray(values) for name, values in predicted_lists.items()}

    landed = previous_value(land_data, "landed", time_s) > 0.5
    valid = (
        ~landed
        & (altitude_agl_m >= arguments.min_altitude_agl)
        & (airspeed >= arguments.min_airspeed)
        & (qbar >= arguments.min_qbar)
        & np.all(np.isfinite(acceleration_frd), axis=1)
        & np.all(np.isfinite(angular), axis=1)
        & accel_bias_valid
    )
    if not arguments.include_propulsion_clamp:
        valid &= ~prop_clamped

    dt = float(np.median(np.diff(time_s)))
    window_samples = max(1, int(round(arguments.smoothing_window / dt)))
    inverse_filtered = {}
    predicted_filtered = {}
    for name in COEFFICIENTS:
        raw_inverse = np.where(valid, inverse[name], np.nan)
        raw_predicted = np.where(valid, predicted[name], np.nan)
        inverse_filtered[name] = nan_smooth(raw_inverse, window_samples)
        predicted_filtered[name] = nan_smooth(raw_predicted, window_samples)

    filtered_valid = valid.copy()
    edge = window_samples // 2
    if edge:
        valid_indices = np.flatnonzero(valid)
        if len(valid_indices):
            filtered_valid[: valid_indices[0] + edge] = False
            filtered_valid[max(0, valid_indices[-1] - edge + 1) :] = False

    columns: Dict[str, Iterable[float]] = {
        "time_s": time_s,
        "analysis_valid": valid.astype(int),
        "landed": landed.astype(int),
        "north_m": position_ned[:, 0], "east_m": position_ned[:, 1], "down_m": position_ned[:, 2],
        "altitude_m": altitude_m, "altitude_agl_m": altitude_agl_m,
        "u_mps": velocity_frd[:, 0], "v_mps": velocity_frd[:, 1], "w_mps": velocity_frd[:, 2],
        "airspeed_mps": airspeed, "alpha_deg": np.degrees(alpha_rad), "beta_deg": np.degrees(beta_rad),
        "alpha_dot_rad_s": alpha_dot, "beta_dot_rad_s": beta_dot,
        "p_rad_s": omega_frd[:, 0], "q_rad_s": omega_frd[:, 1], "r_rad_s": omega_frd[:, 2],
        "pdot_rad_s2": omega_dot_frd[:, 0], "qdot_rad_s2": omega_dot_frd[:, 1], "rdot_rad_s2": omega_dot_frd[:, 2],
        "specific_force_x_mps2": acceleration_frd[:, 0], "specific_force_y_mps2": acceleration_frd[:, 1], "specific_force_z_mps2": acceleration_frd[:, 2],
        "vehicle_acceleration_x_mps2": vehicle_acceleration_frd[:, 0],
        "vehicle_acceleration_y_mps2": vehicle_acceleration_frd[:, 1],
        "vehicle_acceleration_z_mps2": vehicle_acceleration_frd[:, 2],
        "estimated_accel_bias_x_mps2": accel_bias_frd[:, 0],
        "estimated_accel_bias_y_mps2": accel_bias_frd[:, 1],
        "estimated_accel_bias_z_mps2": accel_bias_frd[:, 2],
        "groundtruth_specific_force_x_mps2": groundtruth_specific_force_frd[:, 0],
        "groundtruth_specific_force_y_mps2": groundtruth_specific_force_frd[:, 1],
        "groundtruth_specific_force_z_mps2": groundtruth_specific_force_frd[:, 2],
        "rho_used_kg_m3": rho, "rho_logged_kg_m3": logged_rho, "rho_model_kg_m3": model_rho, "qbar_pa": qbar,
        "rho_plugin_kg_m3": diagnostic_rho,
        "delta_a_doc_deg": delta_doc[:, 0], "delta_e_doc_deg": delta_doc[:, 1], "delta_r_doc_deg": delta_doc[:, 2], "delta_c_doc_deg": delta_doc[:, 3],
        "plugin_delta_a_doc_deg": plugin_reported_delta[:, 0],
        "plugin_delta_e_doc_deg": plugin_reported_delta[:, 1],
        "plugin_delta_r_doc_deg": plugin_reported_delta[:, 2],
        "plugin_delta_c_doc_deg": plugin_reported_delta[:, 3],
        "throttle_target": motor_command, "throttle_state": throttle_state,
        "rpm": rpm, "thrust_n": thrust, "engine_torque_nm": engine_torque, "propulsion_clamped": prop_clamped.astype(int),
        "plugin_throttle_target": plugin_propulsion[:, 0], "plugin_throttle_state": plugin_propulsion[:, 1],
        "plugin_propulsion_altitude_m": plugin_propulsion[:, 2], "plugin_propulsion_airspeed_mps": plugin_propulsion[:, 3],
        "plugin_rpm": plugin_propulsion[:, 4], "plugin_thrust_n": plugin_propulsion[:, 5],
        "plugin_engine_torque_nm": plugin_propulsion[:, 6], "plugin_fuel_rate": plugin_propulsion[:, 7],
        "aero_diagnostic_flags": aero_flags, "propulsion_diagnostic_flags": propulsion_flags,
        "aero_force_x_frd_n": aero_force_frd[:, 0], "aero_force_y_frd_n": aero_force_frd[:, 1], "aero_force_z_frd_n": aero_force_frd[:, 2],
        "aero_moment_x_frd_nm": aero_moment_frd[:, 0], "aero_moment_y_frd_nm": aero_moment_frd[:, 1], "aero_moment_z_frd_nm": aero_moment_frd[:, 2],
    }
    for index in range(8):
        columns[f"theta_joint_{index}_deg"] = theta_deg[:, index]
    for name in COEFFICIENTS:
        coefficient_index = COEFFICIENTS.index(name)
        columns[f"{name}_inverse_raw"] = inverse[name]
        columns[f"{name}_model_raw"] = predicted[name]
        columns[f"{name}_plugin_raw"] = plugin_coefficients[:, coefficient_index]
        columns[f"{name}_inverse_filtered"] = inverse_filtered[name]
        columns[f"{name}_model_filtered"] = predicted_filtered[name]
        columns[f"{name}_residual_filtered"] = inverse_filtered[name] - predicted_filtered[name]
    frame = pd.DataFrame(columns)

    arguments.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = arguments.output_dir / "honghu_v8_aero_coefficient_timeseries.csv"
    json_path = arguments.output_dir / "honghu_v8_aero_coefficient_summary.json"
    plot_path = arguments.output_dir / "honghu_v8_aero_coefficient_comparison.png"
    states_path = arguments.output_dir / "honghu_v8_aero_coefficient_inputs.png"
    frame.to_csv(csv_path, index=False)
    plot_coefficients(frame, plot_path)
    plot_states(frame, states_path)

    plan_item_count = None
    if arguments.plan.exists():
        plan_payload = json.loads(arguments.plan.read_text(encoding="utf-8-sig"))
        plan_item_count = len(plan_payload.get("mission", {}).get("items", []))

    summary: Dict[str, object] = {
        "ulog": str(arguments.ulog.resolve()),
        "ulog_sha256": sha256(arguments.ulog),
        "ulog_build": {
            key: ulog.msg_info_dict.get(key)
            for key in ("ver_sw", "ver_sw_branch", "ver_hw", "sys_os_name", "sys_toolchain")
        },
        "ulog_parameters": {
            name: ulog.initial_parameters.get(name)
            for name in parameter_names
            if name in ulog.initial_parameters
        },
        "plan": str(arguments.plan.resolve()) if arguments.plan.exists() else None,
        "plan_sha256": sha256(arguments.plan) if arguments.plan.exists() else None,
        "mission": {
            "plan_item_count": plan_item_count,
            "ulog_seq_total_max": int(np.max(mission_data.data["seq_total"])),
            "ulog_seq_current_max": int(np.max(mission_data.data["seq_current"])),
            "ulog_seq_reached_max": int(np.max(mission_data.data["seq_reached"])),
            "ulog_finished": bool(np.any(mission_data.data["finished"])),
            "ulog_failure": bool(np.any(mission_data.data["failure"])),
        },
        "method": {
            "mass_kg": MASS_KG,
            "area_m2": AREA_M2,
            "span_m": SPAN_M,
            "mac_m": MAC_M,
            "inertia_frd_kgm2": INERTIA_FRD_KGM2.tolist(),
            "specific_force_relation": (
                "F_aero = mass * (vehicle_acceleration_FRD + "
                "estimator_sensor_bias_FRD) - F_propulsion"
            ),
            "acceleration_source": acceleration_source,
            "gazebo_gravity_m_s2": GAZEBO_GRAVITY_MPS2,
            "moment_relation": "M_aero = I*omega_dot + omega_cross_(I*omega) - M_propulsion",
            "density_source": arguments.density_source,
            "smoothing_window_s": arguments.smoothing_window,
            "surface_source": surface_source,
            "propulsion_source": propulsion_source,
            "wind": "zero, as configured by honghu_v8.sdf",
        },
        "selection": {
            "time_start_s": float(time_s[valid][0]) if np.any(valid) else None,
            "time_end_s": float(time_s[valid][-1]) if np.any(valid) else None,
            "samples": int(np.count_nonzero(valid)),
            "duration_s": float(time_s[valid][-1] - time_s[valid][0]) if np.any(valid) else 0.0,
            "min_airspeed_mps": arguments.min_airspeed,
            "min_altitude_agl_m": arguments.min_altitude_agl,
            "min_qbar_pa": arguments.min_qbar,
            "propulsion_clamped_samples": int(np.count_nonzero(valid & prop_clamped)),
            "airspeed_range_mps": [float(np.min(airspeed[valid])), float(np.max(airspeed[valid]))] if np.any(valid) else None,
            "alpha_range_deg": [float(np.min(np.degrees(alpha_rad[valid]))), float(np.max(np.degrees(alpha_rad[valid])))] if np.any(valid) else None,
            "beta_range_deg": [float(np.min(np.degrees(beta_rad[valid]))), float(np.max(np.degrees(beta_rad[valid])))] if np.any(valid) else None,
        },
        "metrics_raw": {name: metrics(inverse[name], predicted[name], valid) for name in COEFFICIENTS},
        "metrics_filtered": {
            name: metrics(inverse_filtered[name], predicted_filtered[name], filtered_valid)
            for name in COEFFICIENTS
        },
        "outputs": {
            "csv": str(csv_path.resolve()),
            "comparison_plot": str(plot_path.resolve()),
            "inputs_plot": str(states_path.resolve()),
        },
        "diagnostic_truth_available": {
            "aerodynamics": aero_diagnostic is not None,
            "propulsion": propulsion_diagnostic is not None,
        },
        "limitations": [
            "The comparison excludes ground contact and does not perform parameter identification or fit the forward model to the inversion result.",
        ],
    }
    acceleration_residual = acceleration_frd - groundtruth_specific_force_frd
    summary["physical_specific_force_crosscheck"] = {
        "reference": (
            "vehicle_local_position_groundtruth NED acceleration minus Gazebo "
            "gravity, rotated into FRD"
        ),
        "mean_error_m_s2": [
            float(np.mean(acceleration_residual[valid, axis])) for axis in range(3)
        ] if np.any(valid) else None,
        "rmse_m_s2": [
            float(np.sqrt(np.mean(acceleration_residual[valid, axis] ** 2)))
            for axis in range(3)
        ] if np.any(valid) else None,
        "estimated_bias_mean_m_s2": [
            float(np.mean(accel_bias_frd[valid, axis])) for axis in range(3)
        ] if np.any(valid) else None,
    }
    if aero_diagnostic is None:
        summary["limitations"].extend(
            [
                "This historical ULog predates V8 diagnostic logging, so commanded servo angles substitute for Gazebo joint feedback.",
                "alpha_dot and beta_dot are reconstructed from ground-truth pose velocity at logger bandwidth.",
            ]
        )
    else:
        summary["diagnostic_timing"] = {
            "aerodynamics": diagnostic_timing(aero_diagnostic),
            "propulsion": diagnostic_timing(propulsion_diagnostic)
            if propulsion_diagnostic is not None else None,
        }
        delta_residual = plugin_reported_delta - delta_doc
        summary["actual_joint_feedback"] = {
            "delta_doc_formula_max_abs_error_deg": float(np.nanmax(np.abs(delta_residual))),
            "elevator_pair_max_abs_error_deg": float(
                np.nanmax(np.abs(theta_deg[:, 2] - theta_deg[:, 3]))
            ),
            "canard_pair_max_abs_error_deg": float(
                np.nanmax(np.abs(theta_deg[:, 6] - theta_deg[:, 7]))
            ),
        }
        summary["metrics_forward_model_vs_plugin"] = {
            name: metrics(predicted[name], plugin_coefficients[:, index], valid)
            for index, name in enumerate(COEFFICIENTS)
        }
        summary["metrics_inversion_vs_plugin"] = {
            name: metrics(inverse[name], plugin_coefficients[:, index], valid)
            for index, name in enumerate(COEFFICIENTS)
        }
    if propulsion_diagnostic is not None:
        summary["metrics_propulsion_model_vs_plugin"] = {
            "rpm": metrics(rpm, plugin_propulsion[:, 4], valid),
            "thrust_n": metrics(thrust, plugin_propulsion[:, 5], valid),
            "torque_nm": metrics(engine_torque, plugin_propulsion[:, 6], valid),
        }
    json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    summary["outputs"]["json"] = str(json_path.resolve())
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", type=Path)
    parser.add_argument("--plan", type=Path, default=DEFAULT_PLAN)
    parser.add_argument("--output-dir", type=Path, default=ROOT / "analysis_outputs/honghu_v8_aero_coefficient_validation")
    parser.add_argument("--start", type=float, help="optional absolute simulation start time [s]")
    parser.add_argument("--end", type=float, help="optional absolute simulation end time [s]")
    parser.add_argument("--min-airspeed", type=float, default=20.0)
    parser.add_argument("--min-altitude-agl", type=float, default=5.0)
    parser.add_argument("--min-qbar", type=float, default=200.0)
    parser.add_argument("--smoothing-window", type=float, default=0.5)
    parser.add_argument("--density-source", choices=("model", "logged"), default="model")
    parser.add_argument("--include-propulsion-clamp", action="store_true")
    parser.add_argument(
        "--allow-commanded-surface-fallback", action="store_true",
        help=(
            "allow historical logs without honghu_v8_aero_state to reconstruct "
            "surface angles from actuator commands; disabled by default because "
            "aerodynamic validation requires actual Gazebo joint feedback"
        ),
    )
    parser.add_argument(
        "--allow-estimator-biased-acceleration", action="store_true",
        help=(
            "allow historical logs without estimator_sensor_bias to use "
            "vehicle_acceleration directly; disabled by default because the "
            "EKF has already removed a bias estimate from that signal"
        ),
    )
    arguments = parser.parse_args()
    arguments.ulog = arguments.ulog.resolve()
    arguments.plan = arguments.plan.resolve()
    arguments.output_dir = arguments.output_dir.resolve()
    if not arguments.ulog.exists():
        parser.error(f"ULog does not exist: {arguments.ulog}")
    analyze(arguments)


if __name__ == "__main__":
    main()
