#!/usr/bin/env python3
"""Validate Gazebo/PX4 magnetometer frames against logged Gazebo truth.

For a corrected/custom source, pass the known NED field in gauss. For the
native Harmonic compatibility path, use --infer-harmonic-field to reconstruct
the constant field that Gazebo placed in its historical NED world vector.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from pyulog import ULog


def dataset(ulog, name, multi_id=0):
    return next(item for item in ulog.data_list
                if item.name == name and item.multi_id == multi_id)


def rotations(quaternions):
    w, x, y, z = quaternions.T
    matrix = np.empty((len(quaternions), 3, 3))
    matrix[:, 0, 0] = 1 - 2 * (y * y + z * z)
    matrix[:, 0, 1] = 2 * (x * y - z * w)
    matrix[:, 0, 2] = 2 * (x * z + y * w)
    matrix[:, 1, 0] = 2 * (x * y + z * w)
    matrix[:, 1, 1] = 1 - 2 * (x * x + z * z)
    matrix[:, 1, 2] = 2 * (y * z - x * w)
    matrix[:, 2, 0] = 2 * (x * z - y * w)
    matrix[:, 2, 1] = 2 * (y * z + x * w)
    matrix[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return matrix


def euler_deg(quaternions):
    w, x, y, z = quaternions.T
    return np.degrees(np.column_stack((
        np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)),
        np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    )))


def nearest_indices(reference_time, query_time):
    indices = np.searchsorted(reference_time, query_time)
    indices = np.clip(indices, 1, len(reference_time) - 1)
    use_previous = np.abs(reference_time[indices - 1] - query_time) < np.abs(
        reference_time[indices] - query_time)
    return indices - use_previous.astype(int)


def wrap_degrees(values):
    return (values + 180.0) % 360.0 - 180.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("ulog", type=Path)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--field-ned-gauss", nargs=3, type=float,
                       metavar=("NORTH", "EAST", "DOWN"))
    group.add_argument("--infer-harmonic-field", action="store_true")
    parser.add_argument("--settle-seconds", type=float, default=8.0)
    parser.add_argument("--label", default="")
    parser.add_argument("--output-json", type=Path)
    args = parser.parse_args()

    ulog = ULog(str(args.ulog))
    truth = dataset(ulog, "vehicle_attitude_groundtruth")
    estimate = dataset(ulog, "vehicle_attitude")
    magnetometer = dataset(ulog, "vehicle_magnetometer")

    truth_time = truth.data["timestamp"].astype(float) * 1e-6
    mag_time = magnetometer.data["timestamp"].astype(float) * 1e-6
    estimate_time = estimate.data["timestamp"].astype(float) * 1e-6
    truth_q_all = np.column_stack([truth.data[f"q[{i}]"] for i in range(4)])
    mag_q = truth_q_all[nearest_indices(truth_time, mag_time)]
    estimate_q = np.column_stack([estimate.data[f"q[{i}]"] for i in range(4)])
    estimate_truth_q = truth_q_all[nearest_indices(truth_time, estimate_time)]
    measured = np.column_stack([
        magnetometer.data[f"magnetometer_ga[{i}]"] for i in range(3)
    ])
    rotation_frd_to_ned = rotations(mag_q)

    start_time = max(mag_time[0], estimate_time[0], truth_time[0])
    end_time = min(mag_time[-1], estimate_time[-1], truth_time[-1])
    valid = ((mag_time >= start_time + args.settle_seconds)
             & (mag_time <= end_time - 1.0)
             & np.isfinite(measured).all(axis=1))

    inferred_std = None
    if args.infer_harmonic_field:
        # PX4 official Harmonic callback: reported = K * raw_FLU.
        body_flu_legacy = np.column_stack((-measured[:, 1], -measured[:, 0], measured[:, 2]))
        frd_to_flu = np.diag([1.0, -1.0, -1.0])
        ned_to_enu = np.array([[0.0, 1.0, 0.0],
                               [1.0, 0.0, 0.0],
                               [0.0, 0.0, -1.0]])
        rotation_flu_to_enu = np.einsum(
            "ij,njk,kl->nil", ned_to_enu, rotation_frd_to_ned, frd_to_flu)
        inferred_samples = np.einsum(
            "nij,nj->ni", rotation_flu_to_enu, body_flu_legacy)
        field_ned = np.mean(inferred_samples[valid], axis=0)
        inferred_std = np.std(inferred_samples[valid], axis=0)
    else:
        field_ned = np.asarray(args.field_ned_gauss, dtype=float)

    expected = np.einsum("nji,j->ni", rotation_frd_to_ned, field_ned)
    cosine = np.sum(measured * expected, axis=1) / (
        np.linalg.norm(measured, axis=1) * np.linalg.norm(expected, axis=1))
    field_angle_error = np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))

    estimate_euler = euler_deg(estimate_q)
    truth_euler = euler_deg(estimate_truth_q)
    attitude_error = estimate_euler - truth_euler
    attitude_error[:, 2] = wrap_degrees(attitude_error[:, 2])
    estimate_valid = ((estimate_time >= start_time + args.settle_seconds)
                      & (estimate_time <= end_time - 1.0)
                      & np.isfinite(attitude_error).all(axis=1))
    truth_euler_valid = truth_euler[estimate_valid]

    field_norm = float(np.linalg.norm(field_ned))
    declination = math.degrees(math.atan2(field_ned[1], field_ned[0]))
    inclination = math.degrees(math.atan2(
        field_ned[2], math.hypot(field_ned[0], field_ned[1])))
    result = {
        "label": args.label,
        "ulog": str(args.ulog.resolve()),
        "mode": "native_harmonic_inferred" if args.infer_harmonic_field else "known_ned_field",
        "field_ned_gauss": field_ned.tolist(),
        "field_strength_gauss": field_norm,
        "field_declination_deg": declination,
        "field_inclination_deg": inclination,
        "inferred_field_axis_std_gauss": None if inferred_std is None else inferred_std.tolist(),
        "field_angle_error_deg": {
            "mean": float(np.mean(field_angle_error[valid])),
            "p95": float(np.percentile(field_angle_error[valid], 95)),
            "max": float(np.max(field_angle_error[valid])),
        },
        "attitude_estimate_minus_truth_deg": {
            "mean_rpy": np.mean(attitude_error[estimate_valid], axis=0).tolist(),
            "median_rpy": np.median(attitude_error[estimate_valid], axis=0).tolist(),
            "p95_abs_rpy": np.percentile(
                np.abs(attitude_error[estimate_valid]), 95, axis=0).tolist(),
        },
        "truth_attitude_coverage_deg": {
            "min_rpy": np.min(truth_euler_valid, axis=0).tolist(),
            "max_rpy": np.max(truth_euler_valid, axis=0).tolist(),
        },
        "sample_count": int(np.count_nonzero(valid)),
        "pass_field_direction": bool(np.percentile(field_angle_error[valid], 95) <= 0.5),
    }

    print(json.dumps(result, indent=2, ensure_ascii=False))
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(
            json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
