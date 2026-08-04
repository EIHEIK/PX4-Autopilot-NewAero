#!/usr/bin/env python3
"""Compare Xiangyi simulation data with the Honghu V8 aerodynamic model.

The Xiangyi CSV is treated as an external, read-only data source.  Its RFU
specific force and body rates are converted to the PDF/PX4 FRD convention,
then the six aerodynamic coefficients are reconstructed from the rigid-body
equations.  The same flight state and commanded-equivalent surface angles are
evaluated with the independent Honghu V8 Python aerodynamic model.

The file currently contains no non-zero physical servo feedback or engine RPM.
Consequently the comparison is conditional on the documented assumptions:
FW_* / Canard are direct document-angle commands and propulsion is the V8
table model.  No aerodynamic parameters are fitted in this program.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from dataclasses import dataclass
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
DEFAULT_XIANGYI = Path("/home/fly/px4_reference_docs/current/翔仪飞控仿真结果.csv")
DEFAULT_ULOG = ROOT / "build/px4_sitl_default/rootfs/log/2026-07-21/05_22_12.ulg"
DEFAULT_PLAN = Path("/home/fly/px4_reference_docs/current/模仿XY航线规划.plan")
DEFAULT_OUTPUT = ROOT / "analysis_outputs/honghu_v8_xiangyi_comparison"

COEFFICIENTS = ("CL", "CD", "CY", "Cl", "Cm", "Cn")
MASS_KG = 100.0
AREA_M2 = 2.42
SPAN_M = 3.96
MAC_M = 0.62
NOMINAL_DT_S = 0.05
THRUST_DOWN_RAD = math.radians(3.0)
ENGINE_POINT_FRD_M = np.array([-1.23, 0.0, -0.12])
# Word table 3 provides complete inertia data at 73 kg and 150 kg.  The
# Xiangyi 100 kg comparison uses a traceable linear interpolation between the
# two supplied states; it is derived data rather than a direct 100 kg test.
_INERTIA_73 = np.array([25.33, 30.81, 50.98, -0.021, -2.592, -0.0002])
_INERTIA_150 = np.array([25.86, 39.14, 59.12, -0.017, -3.520, -0.0019])
_INERTIA_100 = _INERTIA_73 + ((MASS_KG - 73.0) / (150.0 - 73.0)) * (
    _INERTIA_150 - _INERTIA_73
)
INERTIA_FRD_KGM2 = np.array(
    [
        [_INERTIA_100[0], _INERTIA_100[3], _INERTIA_100[4]],
        [_INERTIA_100[3], _INERTIA_100[1], _INERTIA_100[5]],
        [_INERTIA_100[4], _INERTIA_100[5], _INERTIA_100[2]],
    ]
)

XIANGYI_COLUMNS = (
    "IndexPro", "Second", "Millisecond",
    "Lat", "Lon", "NavHeight", "NavAltitude",
    "V_east", "V_north", "Vz", "IAS", "TAS",
    "Yaw", "Pitch", "Roll", "wX", "wY", "wZ",
    "Acc_X", "Acc_Y", "Acc_Z", "FlyStage",
    "FW_Ail", "FW_Ele", "FW_Thr", "FW_Rud", "Canard",
    "SteeringGear1Angle", "SteeringGear2Angle",
    "SteeringGear3Angle", "SteeringGear4Angle",
    "SteeringGear5Angle", "SteeringGear6Angle",
    "SteeringGear7Angle", "SteeringGear8Angle",
    "SteeringGear9Angle", "SteeringGear10Angle",
    "EngineSpeed",
)


@dataclass(frozen=True)
class FlightSegment:
    name: str
    start: int
    end: int
    airborne_start: int
    airborne_end: int
    plan_coverage_fraction: float
    maximum_position_jump_m: float
    complete: bool
    rejection_reason: str | None


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def finite_float(value: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def json_safe(value):
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, (np.floating, float)):
        return float(value) if math.isfinite(float(value)) else None
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, (np.integer, int)):
        return int(value)
    return value


def write_json(path: Path, payload: object) -> None:
    path.write_text(
        json.dumps(json_safe(payload), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def load_plan_points(path: Path) -> np.ndarray:
    payload = json.loads(path.read_text(encoding="utf-8-sig"))
    points = []
    for item in payload["mission"]["items"]:
        if item.get("command") not in (16, 21, 22):
            continue
        latitude, longitude, altitude = item["params"][4:7]
        if abs(latitude) < 1e-9 or abs(longitude) < 1e-9:
            continue
        points.append((float(latitude), float(longitude), float(altitude)))
    if len(points) < 2:
        raise ValueError(f"mission contains fewer than two geographic points: {path}")
    return np.asarray(points)


def load_xiangyi_csv(path: Path) -> tuple[pd.DataFrame, dict]:
    with path.open(newline="", encoding="gb18030") as handle:
        reader = csv.reader(handle)
        header = next(reader)
        chinese_header = next(reader)
        if len(header) != 577 or len(chinese_header) != len(header):
            raise ValueError(
                f"unexpected Xiangyi schema: {len(header)} English and "
                f"{len(chinese_header)} Chinese columns"
            )
        indices = {}
        for name in XIANGYI_COLUMNS:
            matches = [index for index, candidate in enumerate(header) if candidate == name]
            if name in ("Lat", "Lon") and len(matches) >= 1:
                # The later duplicate belongs to the vision-navigation packet.
                # The first pair is the primary fused-navigation solution used
                # with V_east/V_north/Vz and the rest of this analysis.
                indices[name] = matches[0]
                continue
            if len(matches) != 1:
                raise ValueError(f"expected one Xiangyi column {name!r}, found {matches}")
            indices[name] = matches[0]

        rows: Dict[str, list[float]] = {name: [] for name in XIANGYI_COLUMNS}
        row_lengths: Dict[int, int] = {}
        for row in reader:
            row_lengths[len(row)] = row_lengths.get(len(row), 0) + 1
            for name, index in indices.items():
                rows[name].append(finite_float(row[index]) if index < len(row) else math.nan)

    frame = pd.DataFrame(rows)
    frame.insert(0, "source_row", np.arange(len(frame), dtype=int))
    duplicate_headers = {
        name: header.count(name) for name in sorted(set(header)) if header.count(name) > 1
    }
    metadata = {
        "encoding": "gb18030",
        "english_header_columns": len(header),
        "chinese_header_columns": len(chinese_header),
        "data_rows": len(frame),
        "row_length_counts": row_lengths,
        "duplicate_headers": duplicate_headers,
        "selected_column_indices": indices,
        "selected_chinese_labels": {
            name: chinese_header[index] for name, index in indices.items()
        },
    }
    return frame, metadata


def geodetic_to_enu(
    latitude_deg: np.ndarray,
    longitude_deg: np.ndarray,
    reference_latitude_deg: float,
    reference_longitude_deg: float,
) -> tuple[np.ndarray, np.ndarray]:
    radius = 6378137.0
    reference_latitude_rad = math.radians(reference_latitude_deg)
    east = (
        np.deg2rad(longitude_deg - reference_longitude_deg)
        * radius
        * math.cos(reference_latitude_rad)
    )
    north = np.deg2rad(latitude_deg - reference_latitude_deg) * radius
    return east, north


def distance_m(
    latitude_a: np.ndarray,
    longitude_a: np.ndarray,
    latitude_b: float,
    longitude_b: float,
) -> np.ndarray:
    east, north = geodetic_to_enu(
        latitude_a, longitude_a, latitude_b, longitude_b
    )
    return np.hypot(east, north)


def contiguous_true_ranges(mask: np.ndarray) -> list[tuple[int, int]]:
    starts = np.flatnonzero(mask & ~np.r_[False, mask[:-1]])
    ends = np.flatnonzero(mask & ~np.r_[mask[1:], False])
    return [(int(start), int(end)) for start, end in zip(starts, ends)]


def identify_flights(frame: pd.DataFrame, plan_points: np.ndarray) -> list[FlightSegment]:
    moving_ranges = [
        (start, end)
        for start, end in contiguous_true_ranges(frame["TAS"].to_numpy() > 1.0)
        if end - start + 1 >= 1000
    ]
    flights = []
    for number, (start, end) in enumerate(moving_ranges, 1):
        block = frame.iloc[start:end + 1]
        valid_geo = (
            np.isfinite(block["Lat"])
            & np.isfinite(block["Lon"])
            & (np.abs(block["Lat"]) > 1.0)
            & (np.abs(block["Lon"]) > 1.0)
        )
        geo = block.loc[valid_geo]
        if len(geo) > 1:
            # Recompute each consecutive step about its own first endpoint.
            jumps = []
            lat = geo["Lat"].to_numpy()
            lon = geo["Lon"].to_numpy()
            for index in range(1, len(lat)):
                jumps.append(float(distance_m(
                    np.array([lat[index]]), np.array([lon[index]]),
                    lat[index - 1], lon[index - 1],
                )[0]))
            maximum_jump = max(jumps, default=math.inf)
        else:
            maximum_jump = math.inf

        coverage = []
        track_lat = geo["Lat"].to_numpy()
        track_lon = geo["Lon"].to_numpy()
        for latitude, longitude, _ in plan_points:
            minimum = (
                float(np.min(distance_m(track_lat, track_lon, latitude, longitude)))
                if len(track_lat) else math.inf
            )
            coverage.append(minimum <= 350.0)
        coverage_fraction = float(np.mean(coverage))

        airborne = (
            (block["TAS"].to_numpy() >= 20.0)
            & (block["NavHeight"].to_numpy() >= 5.0)
            & valid_geo.to_numpy()
        )
        airborne_indices = np.flatnonzero(airborne)
        airborne_start = start + int(airborne_indices[0]) if len(airborne_indices) else start
        airborne_end = start + int(airborne_indices[-1]) if len(airborne_indices) else end
        complete = (
            coverage_fraction >= 0.80
            and maximum_jump < 5000.0
            and len(airborne_indices) >= 1000
        )
        reasons = []
        if coverage_fraction < 0.80:
            reasons.append(f"mission coverage {coverage_fraction:.3f} < 0.80")
        if maximum_jump >= 5000.0:
            reasons.append(f"position jump {maximum_jump:.1f} m")
        if len(airborne_indices) < 1000:
            reasons.append(f"only {len(airborne_indices)} airborne samples")
        flights.append(
            FlightSegment(
                name=f"flight_{number}",
                start=start,
                end=end,
                airborne_start=airborne_start,
                airborne_end=airborne_end,
                plan_coverage_fraction=coverage_fraction,
                maximum_position_jump_m=maximum_jump,
                complete=complete,
                rejection_reason="; ".join(reasons) if reasons else None,
            )
        )
    return flights


def reconstruct_time(frame: pd.DataFrame) -> tuple[np.ndarray, dict]:
    clock = frame["Second"].to_numpy() + frame["Millisecond"].to_numpy() * 1e-3
    raw = np.diff(clock)
    unwrapped = np.where(raw < -30.0, raw + 60.0, raw)
    good = np.isfinite(unwrapped) & (unwrapped >= 0.045) & (unwrapped <= 0.055)
    increments = np.where(good, unwrapped, NOMINAL_DT_S)
    time_s = np.r_[0.0, np.cumsum(increments)]
    counter = frame["IndexPro"].to_numpy(dtype=int)
    counter_step = np.mod(np.diff(counter), 256)
    return time_s, {
        "samples": len(frame),
        "nominal_sample_period_s": NOMINAL_DT_S,
        "measured_dt_median_s": float(np.median(unwrapped[good])) if np.any(good) else None,
        "timestamp_reconstructed_steps": int(np.count_nonzero(~good)),
        "counter_nonunit_steps": int(np.count_nonzero(counter_step != 1)),
        "duration_s": float(time_s[-1]),
    }


def odd_window_samples(window_s: float, dt_s: float) -> int:
    result = max(1, int(round(window_s / max(dt_s, 1e-6))))
    return result if result % 2 else result + 1


def hann_smooth(values: np.ndarray, window_samples: int) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    if window_samples <= 1:
        return values.copy()
    if window_samples % 2 == 0:
        window_samples += 1
    kernel = np.hanning(window_samples)
    if not np.any(kernel):
        kernel = np.ones(window_samples)
    kernel /= kernel.sum()
    pad = window_samples // 2
    padded = np.pad(values, (pad, pad), mode="edge")
    finite = np.isfinite(padded)
    numerator = np.convolve(np.where(finite, padded, 0.0), kernel, mode="valid")
    denominator = np.convolve(finite.astype(float), kernel, mode="valid")
    result = np.full_like(values, np.nan)
    usable = denominator > 0.5
    result[usable] = numerator[usable] / denominator[usable]
    return result


def body_to_ned_matrices(
    roll_rad: np.ndarray, pitch_rad: np.ndarray, yaw_rad: np.ndarray
) -> np.ndarray:
    cr, sr = np.cos(roll_rad), np.sin(roll_rad)
    cp, sp = np.cos(pitch_rad), np.sin(pitch_rad)
    cy, sy = np.cos(yaw_rad), np.sin(yaw_rad)
    matrices = np.empty((len(roll_rad), 3, 3))
    matrices[:, 0, 0] = cy * cp
    matrices[:, 0, 1] = cy * sp * sr - sy * cr
    matrices[:, 0, 2] = cy * sp * cr + sy * sr
    matrices[:, 1, 0] = sy * cp
    matrices[:, 1, 1] = sy * sp * sr + cy * cr
    matrices[:, 1, 2] = sy * sp * cr - cy * sr
    matrices[:, 2, 0] = -sp
    matrices[:, 2, 1] = cp * sr
    matrices[:, 2, 2] = cp * cr
    return matrices


def correlations(first: np.ndarray, second: np.ndarray, mask: np.ndarray) -> list[float]:
    result = []
    for axis in range(first.shape[1]):
        usable = mask & np.isfinite(first[:, axis]) & np.isfinite(second[:, axis])
        if np.count_nonzero(usable) < 3:
            result.append(math.nan)
        else:
            result.append(float(np.corrcoef(first[usable, axis], second[usable, axis])[0, 1]))
    return result


def frame_contract_check(frame: pd.DataFrame, time_s: np.ndarray) -> dict:
    dt = float(np.median(np.diff(time_s)))
    window = odd_window_samples(1.0, dt)
    roll = np.deg2rad(frame["Roll"].to_numpy())
    pitch = np.deg2rad(frame["Pitch"].to_numpy())
    yaw = np.unwrap(np.deg2rad(frame["Yaw"].to_numpy()))
    roll_dot = np.gradient(hann_smooth(roll, window), time_s)
    pitch_dot = np.gradient(hann_smooth(pitch, window), time_s)
    yaw_dot = np.gradient(hann_smooth(yaw, window), time_s)
    euler_rates = np.column_stack(
        (
            roll_dot - yaw_dot * np.sin(pitch),
            pitch_dot * np.cos(roll) + yaw_dot * np.sin(roll) * np.cos(pitch),
            -pitch_dot * np.sin(roll) + yaw_dot * np.cos(roll) * np.cos(pitch),
        )
    )
    logged_rates = np.deg2rad(
        np.column_stack(
            (
                frame["wY"].to_numpy(),
                frame["wX"].to_numpy(),
                -frame["wZ"].to_numpy(),
            )
        )
    )

    velocity_ned = np.column_stack(
        (
            frame["V_north"].to_numpy(),
            frame["V_east"].to_numpy(),
            -frame["Vz"].to_numpy(),
        )
    )
    smoothed_velocity = np.column_stack(
        [hann_smooth(velocity_ned[:, axis], window) for axis in range(3)]
    )
    acceleration_ned = np.column_stack(
        [np.gradient(smoothed_velocity[:, axis], time_s) for axis in range(3)]
    )
    matrices = body_to_ned_matrices(roll, pitch, yaw)
    kinematic_specific_force = np.einsum(
        "nji,nj->ni",
        matrices,
        acceleration_ned - np.array([0.0, 0.0, 9.8]),
    )
    logged_specific_force = 9.8 * np.column_stack(
        (
            frame["Acc_Y"].to_numpy(),
            frame["Acc_X"].to_numpy(),
            -frame["Acc_Z"].to_numpy(),
        )
    )
    velocity_body = np.einsum("nji,nj->ni", matrices, smoothed_velocity)
    speed_body = np.linalg.norm(velocity_body, axis=1)

    mask = (
        (frame["TAS"].to_numpy() >= 20.0)
        & (frame["NavHeight"].to_numpy() >= 5.0)
        & np.all(np.isfinite(euler_rates), axis=1)
        & np.all(np.isfinite(kinematic_specific_force), axis=1)
    )
    edge = np.zeros(len(frame), dtype=bool)
    edge[window:-window] = True
    mask &= edge
    rate_correlation = correlations(euler_rates, logged_rates, mask)
    force_correlation = correlations(
        kinematic_specific_force, logged_specific_force, mask
    )
    speed_rmse = float(np.sqrt(np.mean(
        (speed_body[mask] - frame["TAS"].to_numpy()[mask]) ** 2
    )))
    checks = {
        "p_q_r_correlation_at_least_0p85": bool(
            all(value >= 0.85 for value in rate_correlation)
        ),
        "specific_force_correlation_at_least_0p90": bool(
            all(value >= 0.90 for value in force_correlation)
        ),
        "tas_body_speed_rmse_below_0p1mps": speed_rmse < 0.1,
    }
    return {
        "contract": {
            "source_body": "RFU: X right, Y forward, Z up",
            "target_body": "FRD: X forward, Y right, Z down",
            "omega_frd": "[wY, wX, -wZ] deg/s",
            "specific_force_frd": "g * [Acc_Y, Acc_X, -Acc_Z]",
            "world_velocity_ned": "[V_north, V_east, -Vz]",
        },
        "rate_correlation_p_q_r": rate_correlation,
        "specific_force_correlation_x_y_z": force_correlation,
        "tas_vs_body_velocity_rmse_m_s": speed_rmse,
        "checks": checks,
        "pass": all(checks.values()),
    }


def plugin_angle_rates(
    time_s: np.ndarray,
    alpha_rad: np.ndarray,
    beta_rad: np.ndarray,
    speed_mps: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    alpha_dot = np.zeros_like(alpha_rad)
    beta_dot = np.zeros_like(beta_rad)
    previous_alpha = alpha_rad[0]
    previous_beta = beta_rad[0]
    filtered_alpha = 0.0
    filtered_beta = 0.0
    initialized = False
    for index in range(len(time_s)):
        dt = time_s[index] - time_s[index - 1] if index else 0.0
        if speed_mps[index] < 3.0 or dt <= 0.0 or not initialized:
            previous_alpha = alpha_rad[index]
            previous_beta = beta_rad[index]
            filtered_alpha = 0.0
            filtered_beta = 0.0
            initialized = True
        else:
            delta_alpha = math.remainder(alpha_rad[index] - previous_alpha, 2.0 * math.pi)
            delta_beta = math.remainder(beta_rad[index] - previous_beta, 2.0 * math.pi)
            raw_alpha = float(np.clip(delta_alpha / dt, -10.0, 10.0))
            raw_beta = float(np.clip(delta_beta / dt, -10.0, 10.0))
            gain = dt / (0.05 + dt)
            filtered_alpha += gain * (raw_alpha - filtered_alpha)
            filtered_beta += gain * (raw_beta - filtered_beta)
            previous_alpha = alpha_rad[index]
            previous_beta = beta_rad[index]
        alpha_dot[index] = filtered_alpha
        beta_dot[index] = filtered_beta
    return alpha_dot, beta_dot


def engine_state(time_s: np.ndarray, target: np.ndarray) -> np.ndarray:
    state = np.empty_like(target)
    state[0] = target[0]
    for index in range(1, len(target)):
        dt = max(0.0, time_s[index] - time_s[index - 1])
        tau = 0.5 if target[index] > state[index - 1] else 0.3
        gain = min(dt / tau, 1.0)
        state[index] = state[index - 1] + gain * (target[index] - state[index - 1])
    return state


def control_response_sign_check(
    frame: pd.DataFrame,
    time_s: np.ndarray,
    state: dict[str, np.ndarray],
) -> dict:
    """Check command/rate response signs without changing the assumed mapping."""
    window = odd_window_samples(0.5, float(np.median(np.diff(time_s))))
    rate = state["omega_frd"]
    rate_dot = np.column_stack(
        [
            np.gradient(hann_smooth(rate[:, axis], window), time_s)
            for axis in range(3)
        ]
    )
    valid = (
        (state["airspeed"] >= 20.0)
        & (frame["NavHeight"].to_numpy() >= 5.0)
    )

    def one(command_column: str, axis: int) -> dict:
        command = frame[command_column].to_numpy()
        mask = valid & np.isfinite(command) & np.isfinite(rate[:, axis])
        span = float(np.ptp(command[mask])) if np.count_nonzero(mask) else 0.0
        if np.count_nonzero(mask) < 10 or span < 1.0:
            return {
                "command_span_deg": span,
                "command_vs_same_axis_rate_correlation": None,
                "command_vs_same_axis_acceleration_correlation": None,
                "status": "insufficient_excitation",
            }
        command_correlation = float(np.corrcoef(command[mask], rate[mask, axis])[0, 1])
        acceleration_correlation = float(
            np.corrcoef(command[mask], rate_dot[mask, axis])[0, 1]
        )
        return {
            "command_span_deg": span,
            "command_vs_same_axis_rate_correlation": command_correlation,
            "command_vs_same_axis_acceleration_correlation": acceleration_correlation,
            "status": (
                "supports_direct_sign"
                if command_correlation > 0.1
                else "does_not_support_direct_sign"
            ),
        }

    return {
        "interpretation": (
            "Closed-loop correlation is a sign sanity check only; it is not "
            "an actuator calibration or a causal derivative estimate."
        ),
        "aileron_to_roll": one("FW_Ail", 0),
        "elevator_to_pitch": one("FW_Ele", 1),
        "rudder_to_yaw": one("FW_Rud", 2),
        "canard_to_pitch": one("Canard", 1),
    }


def compute_state(frame: pd.DataFrame, time_s: np.ndarray) -> dict[str, np.ndarray]:
    roll = np.deg2rad(frame["Roll"].to_numpy())
    pitch = np.deg2rad(frame["Pitch"].to_numpy())
    yaw = np.unwrap(np.deg2rad(frame["Yaw"].to_numpy()))
    matrices = body_to_ned_matrices(roll, pitch, yaw)
    velocity_ned = np.column_stack(
        (
            frame["V_north"].to_numpy(),
            frame["V_east"].to_numpy(),
            -frame["Vz"].to_numpy(),
        )
    )
    velocity_body = np.einsum("nji,nj->ni", matrices, velocity_ned)
    airspeed = frame["TAS"].to_numpy()
    alpha = np.arctan2(velocity_body[:, 2], velocity_body[:, 0])
    beta = np.arctan2(
        velocity_body[:, 1],
        np.sqrt(velocity_body[:, 0] ** 2 + velocity_body[:, 2] ** 2),
    )
    omega = np.deg2rad(
        np.column_stack(
            (
                frame["wY"].to_numpy(),
                frame["wX"].to_numpy(),
                -frame["wZ"].to_numpy(),
            )
        )
    )
    alpha_dot, beta_dot = plugin_angle_rates(time_s, alpha, beta, airspeed)
    throttle_target = np.clip(frame["FW_Thr"].to_numpy() / 100.0, 0.0, 1.0)
    return {
        "roll_rad": roll,
        "pitch_rad": pitch,
        "yaw_rad": yaw,
        "velocity_body_frd": velocity_body,
        "airspeed": airspeed,
        "alpha_rad": alpha,
        "beta_rad": beta,
        "omega_frd": omega,
        "alpha_dot": alpha_dot,
        "beta_dot": beta_dot,
        "throttle_target": throttle_target,
        "throttle_state": engine_state(time_s, throttle_target),
    }


def forward_coefficients(
    frame: pd.DataFrame, state: dict[str, np.ndarray], model: HonghuV8AeroModel
) -> np.ndarray:
    result = np.empty((len(frame), len(COEFFICIENTS)))
    alpha_deg = np.rad2deg(state["alpha_rad"])
    beta_deg = np.rad2deg(state["beta_rad"])
    omega = state["omega_frd"]
    for index in range(len(frame)):
        coefficients, _ = model.coefficients(
            alpha_deg=float(alpha_deg[index]),
            beta_deg=float(beta_deg[index]),
            speed_mps=float(state["airspeed"][index]),
            delta_a_deg=float(frame["FW_Ail"].iloc[index]),
            delta_e_deg=float(frame["FW_Ele"].iloc[index]),
            delta_r_deg=float(frame["FW_Rud"].iloc[index]),
            delta_c_deg=float(frame["Canard"].iloc[index]),
            p_rad_s=float(omega[index, 0]),
            q_rad_s=float(omega[index, 1]),
            r_rad_s=float(omega[index, 2]),
            alpha_dot_rad_s=float(state["alpha_dot"][index]),
            beta_dot_rad_s=float(state["beta_dot"][index]),
        )
        result[index] = [getattr(coefficients, name) for name in COEFFICIENTS]
    return result


def reconstruct_coefficients(
    frame: pd.DataFrame,
    time_s: np.ndarray,
    state: dict[str, np.ndarray],
    propulsion_model: HonghuV8PropulsionModel,
    *,
    filter_window_s: float,
    gravity_m_s2: float,
    density_height: str,
    thrust_scale: float,
) -> dict[str, np.ndarray]:
    dt = float(np.median(np.diff(time_s)))
    window = odd_window_samples(filter_window_s, dt)
    omega_filtered = np.column_stack(
        [hann_smooth(state["omega_frd"][:, axis], window) for axis in range(3)]
    )
    omega_dot = np.column_stack(
        [np.gradient(omega_filtered[:, axis], time_s) for axis in range(3)]
    )
    specific_force = gravity_m_s2 * np.column_stack(
        (
            frame["Acc_Y"].to_numpy(),
            frame["Acc_X"].to_numpy(),
            -frame["Acc_Z"].to_numpy(),
        )
    )
    altitude_density = frame[density_height].to_numpy()
    density = np.asarray([isa_density(value) for value in altitude_density])
    qbar = 0.5 * density * state["airspeed"] ** 2

    propulsion = propulsion_model.evaluate_many(
        frame["NavHeight"].to_numpy(),
        state["throttle_state"],
        state["airspeed"],
    )
    thrust = thrust_scale * np.asarray([item.thrust_newton for item in propulsion])
    torque = thrust_scale * np.asarray([item.torque_nm for item in propulsion])
    clamped = np.asarray([item.clamped for item in propulsion], dtype=bool)
    prop_force = np.column_stack(
        (
            thrust * math.cos(THRUST_DOWN_RAD),
            np.zeros_like(thrust),
            thrust * math.sin(THRUST_DOWN_RAD),
        )
    )
    prop_moment = np.cross(np.broadcast_to(ENGINE_POINT_FRD_M, prop_force.shape), prop_force)
    prop_moment[:, 0] -= torque

    aero_force = MASS_KG * specific_force - prop_force
    inertia_omega = omega_filtered @ INERTIA_FRD_KGM2.T
    total_moment = (
        omega_dot @ INERTIA_FRD_KGM2.T
        + np.cross(omega_filtered, inertia_omega)
    )
    aero_moment = total_moment - prop_moment

    alpha = state["alpha_rad"]
    beta = state["beta_rad"]
    ca, sa = np.cos(alpha), np.sin(alpha)
    cb, sb = np.cos(beta), np.sin(beta)
    ex = np.column_stack((ca * cb, sb, sa * cb))
    ey = np.column_stack((-ca * sb, cb, -sa * sb))
    ez = np.column_stack((-sa, np.zeros_like(sa), ca))
    force_denominator = qbar * AREA_M2
    coefficients = np.column_stack(
        (
            -np.einsum("ij,ij->i", aero_force, ez) / force_denominator,
            -np.einsum("ij,ij->i", aero_force, ex) / force_denominator,
            np.einsum("ij,ij->i", aero_force, ey) / force_denominator,
            aero_moment[:, 0] / (force_denominator * SPAN_M),
            aero_moment[:, 1] / (force_denominator * MAC_M),
            aero_moment[:, 2] / (force_denominator * SPAN_M),
        )
    )
    return {
        "inverse": coefficients,
        "specific_force": specific_force,
        "omega_filtered": omega_filtered,
        "omega_dot": omega_dot,
        "density": density,
        "qbar": qbar,
        "thrust": thrust,
        "torque": torque,
        "propulsion_clamped": clamped,
    }


def phase_labels(frame: pd.DataFrame, state: dict[str, np.ndarray]) -> np.ndarray:
    labels = np.full(len(frame), "straight_level", dtype=object)
    vertical_speed = frame["Vz"].to_numpy()
    roll_deg = frame["Roll"].to_numpy()
    yaw_rate_deg_s = np.rad2deg(state["omega_frd"][:, 2])
    labels[(np.abs(roll_deg) >= 3.0) | (np.abs(yaw_rate_deg_s) >= 0.5)] = "turn"
    labels[vertical_speed > 0.5] = "takeoff_climb"
    labels[vertical_speed < -0.5] = "descent_approach"
    return labels


def block_bootstrap(
    residual: np.ndarray, block_samples: int = 20, repetitions: int = 300
) -> dict:
    residual = residual[np.isfinite(residual)]
    block_count = len(residual) // block_samples
    if block_count < 2:
        return {"bias_95ci": [math.nan, math.nan], "rmse_95ci": [math.nan, math.nan]}
    blocks = residual[:block_count * block_samples].reshape(block_count, block_samples)
    sums = blocks.sum(axis=1)
    sum_squares = np.square(blocks).sum(axis=1)
    rng = np.random.default_rng(20260728)
    indices = rng.integers(0, block_count, size=(repetitions, block_count))
    sample_count = block_count * block_samples
    biases = sums[indices].sum(axis=1) / sample_count
    rmses = np.sqrt(sum_squares[indices].sum(axis=1) / sample_count)
    return {
        "bias_95ci": [float(value) for value in np.percentile(biases, [2.5, 97.5])],
        "rmse_95ci": [float(value) for value in np.percentile(rmses, [2.5, 97.5])],
    }


def coefficient_metrics(
    inverse: np.ndarray,
    model: np.ndarray,
    mask: np.ndarray,
    *,
    bootstrap: bool,
) -> dict:
    usable = mask & np.all(np.isfinite(np.column_stack((inverse, model))), axis=1)
    first = inverse[usable]
    second = model[usable]
    residual = first - second
    if len(first) < 3:
        return {"samples": int(len(first))}
    model_rms = float(np.sqrt(np.mean(second ** 2)))
    normalization = max(model_rms, 1e-4)
    model_std = float(np.std(second))
    correlation = (
        float(np.corrcoef(first, second)[0, 1])
        if model_std > 1e-4 and np.std(first) > 1e-12 else math.nan
    )
    slope = (
        float(np.polyfit(second, first, 1)[0])
        if model_std > 1e-4 else math.nan
    )
    result = {
        "samples": int(len(first)),
        "bias_inverse_minus_v8": float(np.mean(residual)),
        "mae": float(np.mean(np.abs(residual))),
        "rmse": float(np.sqrt(np.mean(residual ** 2))),
        "p95_abs_error": float(np.percentile(np.abs(residual), 95)),
        "correlation": correlation,
        "regression_slope_inverse_on_v8": slope,
        "inverse_mean": float(np.mean(first)),
        "v8_mean": float(np.mean(second)),
        "inverse_std": float(np.std(first)),
        "v8_std": model_std,
        "normalization_v8_rms": normalization,
        "normalized_abs_bias": float(abs(np.mean(residual)) / normalization),
        "normalized_rmse": float(np.sqrt(np.mean(residual ** 2)) / normalization),
        "has_sufficient_variation_for_correlation": model_std > 1e-4,
    }
    if bootstrap:
        result.update(block_bootstrap(residual))
    return result


def metrics_by_group(
    frame: pd.DataFrame,
    inverse: np.ndarray,
    forward: np.ndarray,
    valid: np.ndarray,
    phases: np.ndarray,
    *,
    bootstrap: bool,
) -> dict:
    dt = float(np.median(np.diff(frame["time_s"].to_numpy())))
    smoothing = odd_window_samples(0.5, dt)
    inverse_filtered = np.column_stack(
        [hann_smooth(inverse[:, axis], smoothing) for axis in range(6)]
    )
    forward_filtered = np.column_stack(
        [hann_smooth(forward[:, axis], smoothing) for axis in range(6)]
    )
    groups = {"overall": valid}
    for phase in ("takeoff_climb", "straight_level", "turn", "descent_approach"):
        groups[phase] = valid & (phases == phase)
    result = {}
    for group, mask in groups.items():
        result[group] = {
            name: coefficient_metrics(
                inverse_filtered[:, axis],
                forward_filtered[:, axis],
                mask,
                bootstrap=bootstrap,
            )
            for axis, name in enumerate(COEFFICIENTS)
        }
    return {
        "metrics": result,
        "inverse_filtered": inverse_filtered,
        "forward_filtered": forward_filtered,
    }


def classify_coefficients(
    baseline: dict, sensitivity: dict, complete_names: Sequence[str]
) -> dict:
    result = {}
    for coefficient in COEFFICIENTS:
        per_flight = [
            baseline[name]["metrics"]["overall"][coefficient] for name in complete_names
        ]

        def passes(metric: dict) -> bool:
            if metric.get("samples", 0) < 100:
                return False
            correlation_pass = (
                not metric.get("has_sufficient_variation_for_correlation", False)
                or metric.get("correlation", -math.inf) >= 0.9
            )
            return (
                metric.get("normalized_abs_bias", math.inf) <= 0.05
                and metric.get("normalized_rmse", math.inf) <= 0.10
                and correlation_pass
            )

        baseline_pass = all(passes(metric) for metric in per_flight)
        all_sensitivity_pass = True
        all_sensitivity_fail = True
        sensitivity_status = {}
        for scenario, scenario_payload in sensitivity.items():
            scenario_metrics = [
                scenario_payload[name]["overall"][coefficient] for name in complete_names
            ]
            status = all(passes(metric) for metric in scenario_metrics)
            sensitivity_status[scenario] = status
            all_sensitivity_pass &= status
            all_sensitivity_fail &= not status

        signs = [
            math.copysign(1.0, metric["bias_inverse_minus_v8"])
            for metric in per_flight
            if metric.get("samples", 0) >= 100
        ]
        same_bias_sign = len(signs) == len(complete_names) and len(set(signs)) == 1
        sufficient_variation = all(
            metric.get("has_sufficient_variation_for_correlation", False)
            for metric in per_flight
        )
        if baseline_pass and all_sensitivity_pass and sufficient_variation:
            grade = "conditional_consistent"
        elif (
            not baseline_pass
            and all_sensitivity_fail
            and same_bias_sign
            and sufficient_variation
        ):
            grade = "inconsistent_under_current_assumptions"
        else:
            grade = "insufficient_evidence"
        result[coefficient] = {
            "grade": grade,
            "baseline_pass": baseline_pass,
            "all_sensitivity_pass": all_sensitivity_pass,
            "same_bias_sign_in_both_complete_flights": same_bias_sign,
            "sufficient_variation": sufficient_variation,
            "sensitivity_pass": sensitivity_status,
        }
    return result


def sample_time(dataset) -> np.ndarray:
    if "timestamp_sample" in dataset.data:
        candidate = np.asarray(dataset.data["timestamp_sample"], dtype=float)
        if np.count_nonzero(candidate > 0) > len(candidate) // 2:
            return candidate * 1e-6
    return np.asarray(dataset.data["timestamp"], dtype=float) * 1e-6


def get_dataset(ulog: ULog, name: str):
    for dataset in ulog.data_list:
        if dataset.name == name and dataset.multi_id == 0:
            return dataset
    raise KeyError(f"ULog is missing {name}[0]")


def interpolate_dataset(dataset, fields: Sequence[str], target_time: np.ndarray) -> np.ndarray:
    source_time = sample_time(dataset)
    order = np.argsort(source_time)
    source_time = source_time[order]
    return np.column_stack(
        [
            np.interp(
                target_time,
                source_time,
                np.asarray(dataset.data[field], dtype=float)[order],
            )
            for field in fields
        ]
    )


def quaternion_to_euler(quaternion: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    q = quaternion.copy()
    for index in range(1, len(q)):
        if np.dot(q[index - 1], q[index]) < 0.0:
            q[index] *= -1.0
    q /= np.maximum(np.linalg.norm(q, axis=1)[:, None], 1e-12)
    w, x, y, z = q.T
    roll = np.arctan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return roll, pitch, yaw


def load_px4_track(path: Path) -> pd.DataFrame:
    topics = [
        "vehicle_global_position",
        "vehicle_attitude",
        "airspeed_validated",
    ]
    ulog = ULog(str(path), message_name_filter_list=topics)
    global_position = get_dataset(ulog, "vehicle_global_position")
    attitude = get_dataset(ulog, "vehicle_attitude")
    airspeed = get_dataset(ulog, "airspeed_validated")
    time_s = sample_time(global_position)
    latitude = np.asarray(global_position.data["lat"], dtype=float)
    longitude = np.asarray(global_position.data["lon"], dtype=float)
    altitude = np.asarray(global_position.data["alt"], dtype=float)
    quaternion = interpolate_dataset(
        attitude, ("q[0]", "q[1]", "q[2]", "q[3]"), time_s
    )
    roll, pitch, yaw = quaternion_to_euler(quaternion)
    true_airspeed = interpolate_dataset(
        airspeed, ("true_airspeed_m_s",), time_s
    )[:, 0]
    valid_geo = (
        np.isfinite(latitude)
        & np.isfinite(longitude)
        & (np.abs(latitude) > 1.0)
        & (np.abs(longitude) > 1.0)
    )
    ground_altitude = float(np.median(altitude[valid_geo][: min(500, np.count_nonzero(valid_geo))]))
    return pd.DataFrame(
        {
            "time_s": time_s - time_s[0],
            "Lat": latitude,
            "Lon": longitude,
            "height_m": altitude - ground_altitude,
            "airspeed_m_s": true_airspeed,
            "roll_deg": np.rad2deg(roll),
            "pitch_deg": np.rad2deg(pitch),
            "yaw_deg": np.mod(np.rad2deg(yaw), 360.0),
            "valid_geo": valid_geo.astype(int),
        }
    )


def route_geometry(plan_points: np.ndarray) -> dict:
    reference_latitude = float(plan_points[0, 0])
    reference_longitude = float(plan_points[0, 1])
    east, north = geodetic_to_enu(
        plan_points[:, 0],
        plan_points[:, 1],
        reference_latitude,
        reference_longitude,
    )
    route = np.column_stack((east, north))
    vectors = np.diff(route, axis=0)
    lengths = np.linalg.norm(vectors, axis=1)
    cumulative = np.r_[0.0, np.cumsum(lengths)]
    return {
        "reference_latitude": reference_latitude,
        "reference_longitude": reference_longitude,
        "route": route,
        "vectors": vectors,
        "lengths": lengths,
        "cumulative": cumulative,
    }


def project_to_route(east: np.ndarray, north: np.ndarray, geometry: dict) -> dict:
    points = np.column_stack((east, north))
    route = geometry["route"]
    vectors = geometry["vectors"]
    lengths = geometry["lengths"]
    cumulative = geometry["cumulative"]
    best_distance = np.full(len(points), np.inf)
    best_signed = np.full(len(points), np.nan)
    best_progress = np.full(len(points), np.nan)
    best_segment = np.full(len(points), -1, dtype=int)
    for segment, (start, vector, length) in enumerate(zip(route[:-1], vectors, lengths)):
        if length <= 0.0:
            continue
        relative = points - start
        fraction = np.clip((relative @ vector) / (length * length), 0.0, 1.0)
        projection = start + fraction[:, None] * vector
        delta = points - projection
        distance = np.linalg.norm(delta, axis=1)
        signed = (vector[0] * delta[:, 1] - vector[1] * delta[:, 0]) / length
        replace = distance < best_distance
        best_distance[replace] = distance[replace]
        best_signed[replace] = signed[replace]
        best_progress[replace] = cumulative[segment] + fraction[replace] * length
        best_segment[replace] = segment
    return {
        "cross_track_abs_m": best_distance,
        "cross_track_signed_m": best_signed,
        "route_progress_raw_m": best_progress,
        "route_segment": best_segment,
    }


def project_to_route_in_order(
    east: np.ndarray, north: np.ndarray, geometry: dict
) -> dict:
    """Map a flown track to the mission polyline without jumping to later legs.

    The mission returns close to its TAKEOFF point.  A global nearest-segment
    projection can therefore label the first airborne sample as a landing-leg
    sample.  Starting at segment zero and permitting only forward transitions
    preserves mission order while still allowing a missed waypoint or data gap
    to skip one segment.
    """
    points = np.column_stack((east, north))
    route = geometry["route"]
    vectors = geometry["vectors"]
    lengths = geometry["lengths"]
    cumulative = geometry["cumulative"]
    segment_count = len(lengths)
    cross_abs = np.full(len(points), np.nan)
    cross_signed = np.full(len(points), np.nan)
    progress = np.full(len(points), np.nan)
    assigned_segment = np.full(len(points), -1, dtype=int)
    current_segment = 0

    for sample, point in enumerate(points):
        best = None
        final_candidate = min(current_segment + 2, segment_count - 1)
        for segment in range(current_segment, final_candidate + 1):
            length = lengths[segment]
            if length <= 0.0:
                continue
            vector = vectors[segment]
            relative = point - route[segment]
            fraction = float(np.clip(
                np.dot(relative, vector) / (length * length), 0.0, 1.0
            ))
            projection = route[segment] + fraction * vector
            delta = point - projection
            distance = float(np.linalg.norm(delta))
            signed = float(
                (vector[0] * delta[1] - vector[1] * delta[0]) / length
            )
            candidate = (distance, segment, fraction, signed)
            if best is None or candidate[0] < best[0]:
                best = candidate
        if best is None:
            continue
        distance, segment, fraction, signed = best
        current_segment = segment
        cross_abs[sample] = distance
        cross_signed[sample] = signed
        progress[sample] = cumulative[segment] + fraction * lengths[segment]
        assigned_segment[sample] = segment

    return {
        "cross_track_abs_m": cross_abs,
        "cross_track_signed_m": cross_signed,
        "route_progress_raw_m": progress,
        "route_segment": assigned_segment,
    }


def prepare_track(
    name: str,
    frame: pd.DataFrame,
    geometry: dict,
    *,
    height_column: str,
    speed_column: str,
    roll_column: str,
) -> pd.DataFrame:
    valid = (
        np.isfinite(frame["Lat"])
        & np.isfinite(frame["Lon"])
        & np.isfinite(frame[height_column])
        & (np.abs(frame["Lat"]) > 1.0)
        & (np.abs(frame["Lon"]) > 1.0)
        & (frame[height_column] >= 5.0)
    )
    track = frame.loc[valid].copy()
    east, north = geodetic_to_enu(
        track["Lat"].to_numpy(),
        track["Lon"].to_numpy(),
        geometry["reference_latitude"],
        geometry["reference_longitude"],
    )
    # Both simulations start near the later landing leg.  Comparing from that
    # point would let a nearest-polyline search assign the first airborne
    # samples to the end of the mission.  Start at the closest passage of the
    # common TAKEOFF waypoint, as required by the comparison contract.
    takeoff_distance = np.hypot(
        east - geometry["route"][0, 0],
        north - geometry["route"][0, 1],
    )
    # The return / landing path can pass closer to the TAKEOFF coordinate than
    # the climbout did.  Restrict the search to the first third so alignment
    # cannot silently begin on the homebound leg.
    takeoff_search_samples = max(1, len(takeoff_distance) // 3)
    takeoff_index = int(np.argmin(takeoff_distance[:takeoff_search_samples]))
    track = track.iloc[takeoff_index:].copy()
    east = east[takeoff_index:]
    north = north[takeoff_index:]
    projection = project_to_route_in_order(east, north, geometry)
    track["east_m"] = east
    track["north_m"] = north
    track["height_agl_m"] = track[height_column].to_numpy()
    track["airspeed_m_s"] = track[speed_column].to_numpy()
    track["roll_for_comparison_deg"] = track[roll_column].to_numpy()
    for key, values in projection.items():
        track[key] = values
    track["route_progress_m"] = np.maximum.accumulate(
        track["route_progress_raw_m"].to_numpy()
    )
    track["track_name"] = name
    return track


def fit_circle(points: np.ndarray) -> float:
    if len(points) < 10:
        return math.nan
    matrix = np.column_stack((2.0 * points[:, 0], 2.0 * points[:, 1], np.ones(len(points))))
    target = points[:, 0] ** 2 + points[:, 1] ** 2
    try:
        center_x, center_y, constant = np.linalg.lstsq(matrix, target, rcond=None)[0]
    except np.linalg.LinAlgError:
        return math.nan
    radius_squared = constant + center_x ** 2 + center_y ** 2
    return math.sqrt(radius_squared) if radius_squared > 0.0 else math.nan


def track_metrics(track: pd.DataFrame, geometry: dict) -> dict:
    cross = track["cross_track_abs_m"].to_numpy()
    segment_metrics = {}
    for segment in range(len(geometry["lengths"])):
        block = track[track["route_segment"] == segment]
        if len(block) < 2:
            continue
        values = block["cross_track_abs_m"].to_numpy()
        segment_metrics[str(segment)] = {
            "samples": len(block),
            "cross_track_rms_m": float(np.sqrt(np.mean(values ** 2))),
            "cross_track_p95_m": float(np.percentile(values, 95)),
            "cross_track_max_m": float(np.max(values)),
            "mean_height_agl_m": float(block["height_agl_m"].mean()),
            "mean_airspeed_m_s": float(block["airspeed_m_s"].mean()),
            "max_abs_roll_deg": float(np.max(np.abs(block["roll_for_comparison_deg"]))),
            "elapsed_s": float(block["time_s"].iloc[-1] - block["time_s"].iloc[0]),
        }
    turns = {}
    cumulative = geometry["cumulative"]
    route = geometry["route"]
    for waypoint in range(1, len(route) - 1):
        progress = cumulative[waypoint]
        block = track[
            (track["route_progress_m"] >= progress - 400.0)
            & (track["route_progress_m"] <= progress + 400.0)
        ]
        if len(block) < 10:
            continue
        point_distance = np.hypot(
            block["east_m"].to_numpy() - route[waypoint, 0],
            block["north_m"].to_numpy() - route[waypoint, 1],
        )
        before = block[block["route_progress_m"] < progress]
        anticipation = math.nan
        if len(before) >= 3:
            incoming = route[waypoint] - route[waypoint - 1]
            incoming_heading = math.atan2(incoming[1], incoming[0])
            de = np.gradient(before["east_m"].to_numpy())
            dn = np.gradient(before["north_m"].to_numpy())
            heading = np.arctan2(dn, de)
            difference = np.abs(np.arctan2(
                np.sin(heading - incoming_heading),
                np.cos(heading - incoming_heading),
            ))
            changed = np.flatnonzero(difference > math.radians(5.0))
            if len(changed):
                anticipation = float(
                    progress - before["route_progress_m"].iloc[int(changed[0])]
                )
        turns[str(waypoint)] = {
            "minimum_distance_to_waypoint_m": float(np.min(point_distance)),
            "corner_cut_distance_m": float(np.min(point_distance)),
            "equivalent_circle_radius_m": fit_circle(
                block[["east_m", "north_m"]].to_numpy()
            ),
            "turn_anticipation_distance_m": anticipation,
            "max_abs_roll_deg": float(
                np.max(np.abs(block["roll_for_comparison_deg"]))
            ),
        }
    return {
        "samples": len(track),
        "elapsed_s": float(track["time_s"].iloc[-1] - track["time_s"].iloc[0]),
        "route_progress_max_m": float(track["route_progress_m"].max()),
        "cross_track_rms_m": float(np.sqrt(np.mean(cross ** 2))),
        "cross_track_p95_m": float(np.percentile(cross, 95)),
        "cross_track_max_m": float(np.max(cross)),
        "height_range_m": [
            float(track["height_agl_m"].min()),
            float(track["height_agl_m"].max()),
        ],
        "airspeed_range_m_s": [
            float(track["airspeed_m_s"].min()),
            float(track["airspeed_m_s"].max()),
        ],
        "start_airspeed_mean_m_s": float(
            track["airspeed_m_s"].iloc[:min(50, len(track))].mean()
        ),
        "approach_airspeed_mean_m_s": float(
            track["airspeed_m_s"].iloc[-min(50, len(track)):].mean()
        ),
        "segment_metrics": segment_metrics,
        "turn_metrics": turns,
    }


def progress_profile(track: pd.DataFrame, grid: np.ndarray) -> dict[str, np.ndarray]:
    ordered = track.sort_values("route_progress_m")
    progress = ordered["route_progress_m"].to_numpy()
    unique_progress, unique_indices = np.unique(progress, return_index=True)
    result = {}
    for column in (
        "east_m", "north_m", "height_agl_m", "airspeed_m_s",
        "roll_for_comparison_deg",
    ):
        values = ordered[column].to_numpy()[unique_indices]
        result[column] = np.interp(grid, unique_progress, values, left=np.nan, right=np.nan)
    return result


def pair_track_metrics(
    first: pd.DataFrame, second: pd.DataFrame, step_m: float = 100.0
) -> dict:
    low = max(float(first["route_progress_m"].min()), float(second["route_progress_m"].min()))
    high = min(float(first["route_progress_m"].max()), float(second["route_progress_m"].max()))
    grid = np.arange(math.ceil(low / step_m) * step_m, high, step_m)
    if len(grid) < 2:
        return {"progress_bins": 0}
    a = progress_profile(first, grid)
    b = progress_profile(second, grid)
    horizontal = np.hypot(a["east_m"] - b["east_m"], a["north_m"] - b["north_m"])
    height = a["height_agl_m"] - b["height_agl_m"]
    speed = a["airspeed_m_s"] - b["airspeed_m_s"]
    return {
        "progress_bins": len(grid),
        "progress_range_m": [float(grid[0]), float(grid[-1])],
        "horizontal_separation_rms_m": float(np.sqrt(np.nanmean(horizontal ** 2))),
        "horizontal_separation_p95_m": float(np.nanpercentile(horizontal, 95)),
        "height_difference_bias_m": float(np.nanmean(height)),
        "height_difference_rmse_m": float(np.sqrt(np.nanmean(height ** 2))),
        "airspeed_difference_bias_m_s": float(np.nanmean(speed)),
        "airspeed_difference_rmse_m_s": float(np.sqrt(np.nanmean(speed ** 2))),
    }


def plot_coefficients(timeseries: pd.DataFrame, output: Path) -> None:
    fig, axes = plt.subplots(3, 2, figsize=(15, 11), sharex=False)
    colors = {"flight_2": "#0072B2", "flight_3": "#009E73"}
    for axis, coefficient in zip(axes.flat, COEFFICIENTS):
        for flight, block in timeseries.groupby("flight"):
            valid = block["analysis_valid"] > 0
            shown = block.loc[valid]
            axis.plot(
                shown["time_s"],
                shown[f"{coefficient}_inverse_filtered"],
                color=colors.get(flight, "#0072B2"),
                lw=0.9,
                label=f"{flight} inversion",
            )
            axis.plot(
                shown["time_s"],
                shown[f"{coefficient}_v8_filtered"],
                color="#D55E00",
                lw=0.8,
                alpha=0.8,
                label=f"{flight} V8",
            )
        axis.set_ylabel(coefficient)
        axis.set_xlabel("flight-relative time [s]")
        axis.grid(True, alpha=0.3)
    axes[0, 0].legend(fontsize=8, ncol=2)
    fig.suptitle("Xiangyi rigid-body inversion vs Honghu V8 forward model")
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


def plot_residuals(timeseries: pd.DataFrame, output: Path) -> None:
    valid = timeseries[timeseries["analysis_valid"] > 0].copy()
    x_map = {
        "CL": ("alpha_deg", "alpha [deg]"),
        "CD": ("alpha_deg", "alpha [deg]"),
        "CY": ("beta_deg", "beta [deg]"),
        "Cl": ("delta_a_doc_deg", "aileron command [deg]"),
        "Cm": ("delta_e_doc_deg", "elevator command [deg]"),
        "Cn": ("beta_deg", "beta [deg]"),
    }
    fig, axes = plt.subplots(3, 2, figsize=(15, 11), layout="constrained")
    for axis, coefficient in zip(axes.flat, COEFFICIENTS):
        column, label = x_map[coefficient]
        residual = (
            valid[f"{coefficient}_inverse_filtered"]
            - valid[f"{coefficient}_v8_filtered"]
        )
        scatter = axis.scatter(
            valid[column],
            residual,
            c=valid["airspeed_m_s"],
            s=3,
            alpha=0.35,
            cmap="viridis",
        )
        axis.axhline(0.0, color="black", lw=0.7)
        axis.set_xlabel(label)
        axis.set_ylabel(f"{coefficient} inversion - V8")
        axis.grid(True, alpha=0.25)
    fig.suptitle("Aerodynamic coefficient residuals versus operating state")
    fig.colorbar(
        scatter,
        ax=axes.ravel().tolist(),
        label="TAS [m/s]",
        shrink=0.75,
        pad=0.02,
    )
    fig.savefig(output, dpi=170)
    plt.close(fig)


def plot_trajectory(
    tracks: dict[str, pd.DataFrame], geometry: dict, output: Path
) -> None:
    fig = plt.figure(figsize=(16, 11))
    axis_map = fig.add_subplot(2, 2, 1)
    axis_3d = fig.add_subplot(2, 2, 2, projection="3d")
    axis_height = fig.add_subplot(2, 2, 3)
    axis_speed = fig.add_subplot(2, 2, 4)
    colors = {
        "xiangyi_flight_2": "#0072B2",
        "xiangyi_flight_3": "#009E73",
        "px4_v8": "#D55E00",
    }
    route = geometry["route"]
    axis_map.plot(route[:, 0], route[:, 1], "k--", lw=1.0, label="mission polyline")
    axis_3d.plot(
        route[:, 0], route[:, 1], np.asarray([0.0] + list(np.zeros(len(route) - 1))),
        "k--", lw=0.7,
    )
    for name, track in tracks.items():
        color = colors.get(name)
        axis_map.plot(track["east_m"], track["north_m"], color=color, lw=1.0, label=name)
        axis_3d.plot(
            track["east_m"], track["north_m"], track["height_agl_m"],
            color=color, lw=0.8, label=name,
        )
        axis_height.plot(
            track["route_progress_m"], track["height_agl_m"],
            color=color, lw=0.9, label=name,
        )
        axis_speed.plot(
            track["route_progress_m"], track["airspeed_m_s"],
            color=color, lw=0.9, label=name,
        )
    axis_map.set_xlabel("East [m]")
    axis_map.set_ylabel("North [m]")
    axis_map.axis("equal")
    axis_map.grid(True, alpha=0.3)
    axis_map.legend(fontsize=8)
    axis_3d.set_xlabel("East [m]")
    axis_3d.set_ylabel("North [m]")
    axis_3d.set_zlabel("AGL [m]")
    axis_height.set_xlabel("route progress [m]")
    axis_height.set_ylabel("AGL [m]")
    axis_height.grid(True, alpha=0.3)
    axis_speed.set_xlabel("route progress [m]")
    axis_speed.set_ylabel("TAS [m/s]")
    axis_speed.grid(True, alpha=0.3)
    fig.suptitle("Closed-loop route comparison (descriptive, not an aero pass/fail)")
    fig.tight_layout()
    fig.savefig(output, dpi=170)
    plt.close(fig)


def build_timeseries(
    name: str,
    frame: pd.DataFrame,
    time_s: np.ndarray,
    state: dict[str, np.ndarray],
    reconstructed: dict[str, np.ndarray],
    forward: np.ndarray,
    inverse_filtered: np.ndarray,
    forward_filtered: np.ndarray,
    valid: np.ndarray,
    phases: np.ndarray,
) -> pd.DataFrame:
    payload = pd.DataFrame(
        {
            "flight": name,
            "source_row": frame["source_row"].to_numpy(),
            "time_s": time_s,
            "analysis_valid": valid.astype(int),
            "phase": phases,
            "latitude_deg": frame["Lat"].to_numpy(),
            "longitude_deg": frame["Lon"].to_numpy(),
            "altitude_agl_m": frame["NavHeight"].to_numpy(),
            "altitude_msl_m": frame["NavAltitude"].to_numpy(),
            "airspeed_m_s": state["airspeed"],
            "alpha_deg": np.rad2deg(state["alpha_rad"]),
            "beta_deg": np.rad2deg(state["beta_rad"]),
            "p_rad_s": state["omega_frd"][:, 0],
            "q_rad_s": state["omega_frd"][:, 1],
            "r_rad_s": state["omega_frd"][:, 2],
            "pdot_rad_s2": reconstructed["omega_dot"][:, 0],
            "qdot_rad_s2": reconstructed["omega_dot"][:, 1],
            "rdot_rad_s2": reconstructed["omega_dot"][:, 2],
            "specific_force_x_m_s2": reconstructed["specific_force"][:, 0],
            "specific_force_y_m_s2": reconstructed["specific_force"][:, 1],
            "specific_force_z_m_s2": reconstructed["specific_force"][:, 2],
            "delta_a_doc_deg": frame["FW_Ail"].to_numpy(),
            "delta_e_doc_deg": frame["FW_Ele"].to_numpy(),
            "delta_r_doc_deg": frame["FW_Rud"].to_numpy(),
            "delta_c_doc_deg": frame["Canard"].to_numpy(),
            "throttle_target": state["throttle_target"],
            "throttle_state": state["throttle_state"],
            "thrust_n": reconstructed["thrust"],
            "engine_torque_nm": reconstructed["torque"],
            "rho_kg_m3": reconstructed["density"],
            "qbar_pa": reconstructed["qbar"],
        }
    )
    for axis, coefficient in enumerate(COEFFICIENTS):
        payload[f"{coefficient}_inverse_raw"] = reconstructed["inverse"][:, axis]
        payload[f"{coefficient}_inverse_filtered"] = inverse_filtered[:, axis]
        payload[f"{coefficient}_v8_raw"] = forward[:, axis]
        payload[f"{coefficient}_v8_filtered"] = forward_filtered[:, axis]
    return payload


def analyze(arguments: argparse.Namespace) -> dict:
    arguments.output_dir.mkdir(parents=True, exist_ok=True)
    plan_points = load_plan_points(arguments.plan)
    source, source_metadata = load_xiangyi_csv(arguments.xiangyi_csv)
    flights = identify_flights(source, plan_points)
    complete = [flight for flight in flights if flight.complete]
    if len(complete) != 2:
        raise RuntimeError(
            f"expected exactly two complete Xiangyi flights, found "
            f"{[(item.name, item.complete, item.rejection_reason) for item in flights]}"
        )
    if complete[0].name != "flight_2" or complete[1].name != "flight_3":
        raise RuntimeError(f"unexpected complete-flight identities: {complete}")

    aero_model = HonghuV8AeroModel()
    propulsion_model = HonghuV8PropulsionModel()
    baseline: dict = {}
    sensitivity: dict = {
        "filter_0p25s": {},
        "filter_1p0s": {},
        "gravity_9p80665": {},
        "density_navheight": {},
        "thrust_0p8": {},
        "thrust_1p2": {},
    }
    frame_checks = {}
    control_sign_checks = {}
    timeseries_parts = []
    xiangyi_tracks = {}
    geometry = route_geometry(plan_points)

    scenario_parameters = {
        "filter_0p25s": dict(
            filter_window_s=0.25, gravity_m_s2=9.8,
            density_height="NavAltitude", thrust_scale=1.0,
        ),
        "filter_1p0s": dict(
            filter_window_s=1.0, gravity_m_s2=9.8,
            density_height="NavAltitude", thrust_scale=1.0,
        ),
        "gravity_9p80665": dict(
            filter_window_s=0.5, gravity_m_s2=9.80665,
            density_height="NavAltitude", thrust_scale=1.0,
        ),
        "density_navheight": dict(
            filter_window_s=0.5, gravity_m_s2=9.8,
            density_height="NavHeight", thrust_scale=1.0,
        ),
        "thrust_0p8": dict(
            filter_window_s=0.5, gravity_m_s2=9.8,
            density_height="NavAltitude", thrust_scale=0.8,
        ),
        "thrust_1p2": dict(
            filter_window_s=0.5, gravity_m_s2=9.8,
            density_height="NavAltitude", thrust_scale=1.2,
        ),
    }

    for flight in complete:
        block = source.iloc[flight.start:flight.end + 1].copy().reset_index(drop=True)
        time_s, timing = reconstruct_time(block)
        block["time_s"] = time_s
        frame_check = frame_contract_check(block, time_s)
        frame_check["timing"] = timing
        frame_checks[flight.name] = frame_check
        if not frame_check["pass"]:
            raise RuntimeError(f"{flight.name} RFU/FRD contract check failed: {frame_check}")

        state = compute_state(block, time_s)
        control_sign_checks[flight.name] = control_response_sign_check(
            block, time_s, state
        )
        forward = forward_coefficients(block, state, aero_model)
        baseline_reconstruction = reconstruct_coefficients(
            block,
            time_s,
            state,
            propulsion_model,
            filter_window_s=0.5,
            gravity_m_s2=9.8,
            density_height="NavAltitude",
            thrust_scale=1.0,
        )
        phases = phase_labels(block, state)
        geographic_valid = (
            np.isfinite(block["Lat"].to_numpy())
            & np.isfinite(block["Lon"].to_numpy())
            & (np.abs(block["Lat"].to_numpy()) > 1.0)
            & (np.abs(block["Lon"].to_numpy()) > 1.0)
        )
        body_speed = np.linalg.norm(state["velocity_body_frd"], axis=1)
        state_continuous = (
            geographic_valid
            & (np.abs(body_speed - state["airspeed"]) <= 0.5)
            & (np.abs(np.rad2deg(state["alpha_rad"])) <= 20.0)
            & (np.abs(np.rad2deg(state["beta_rad"])) <= 16.0)
            & (np.abs(block["Pitch"].to_numpy()) <= 30.0)
            & (np.abs(block["Roll"].to_numpy()) <= 60.0)
        )
        valid = (
            (state["airspeed"] >= 20.0)
            & (block["NavHeight"].to_numpy() >= 5.0)
            & (baseline_reconstruction["qbar"] >= 200.0)
            & state_continuous
            & np.all(np.isfinite(baseline_reconstruction["inverse"]), axis=1)
            & np.all(np.isfinite(forward), axis=1)
            & ~baseline_reconstruction["propulsion_clamped"]
        )
        grouped = metrics_by_group(
            block,
            baseline_reconstruction["inverse"],
            forward,
            valid,
            phases,
            bootstrap=True,
        )
        baseline[flight.name] = {
            "valid_samples": int(np.count_nonzero(valid)),
            "valid_duration_s": float(np.count_nonzero(valid) * NOMINAL_DT_S),
            "state_discontinuity_samples_excluded": int(
                np.count_nonzero(~state_continuous)
            ),
            "state_envelope": {
                "airspeed_m_s": [
                    float(np.min(state["airspeed"][valid])),
                    float(np.max(state["airspeed"][valid])),
                ],
                "alpha_deg": [
                    float(np.min(np.rad2deg(state["alpha_rad"][valid]))),
                    float(np.max(np.rad2deg(state["alpha_rad"][valid]))),
                ],
                "beta_deg": [
                    float(np.min(np.rad2deg(state["beta_rad"][valid]))),
                    float(np.max(np.rad2deg(state["beta_rad"][valid]))),
                ],
                "roll_deg": [
                    float(np.min(block["Roll"].to_numpy()[valid])),
                    float(np.max(block["Roll"].to_numpy()[valid])),
                ],
            },
            "phase_sample_counts": {
                phase: int(np.count_nonzero(valid & (phases == phase)))
                for phase in ("takeoff_climb", "straight_level", "turn", "descent_approach")
            },
            "metrics": grouped["metrics"],
        }
        timeseries_parts.append(
            build_timeseries(
                flight.name,
                block,
                time_s,
                state,
                baseline_reconstruction,
                forward,
                grouped["inverse_filtered"],
                grouped["forward_filtered"],
                valid,
                phases,
            )
        )

        for scenario, parameters in scenario_parameters.items():
            reconstruction = reconstruct_coefficients(
                block, time_s, state, propulsion_model, **parameters
            )
            scenario_valid = (
                (state["airspeed"] >= 20.0)
                & (block["NavHeight"].to_numpy() >= 5.0)
                & (reconstruction["qbar"] >= 200.0)
                & state_continuous
                & np.all(np.isfinite(reconstruction["inverse"]), axis=1)
                & np.all(np.isfinite(forward), axis=1)
                & ~reconstruction["propulsion_clamped"]
            )
            sensitivity[scenario][flight.name] = metrics_by_group(
                block,
                reconstruction["inverse"],
                forward,
                scenario_valid,
                phases,
                bootstrap=False,
            )["metrics"]

        xiangyi_tracks[f"xiangyi_{flight.name}"] = prepare_track(
            f"xiangyi_{flight.name}",
            block,
            geometry,
            height_column="NavHeight",
            speed_column="TAS",
            roll_column="Roll",
        )

    timeseries = pd.concat(timeseries_parts, ignore_index=True)
    timeseries_path = arguments.output_dir / "xiangyi_coefficient_timeseries.csv"
    timeseries.to_csv(timeseries_path, index=False, float_format="%.9g")

    px4_source = load_px4_track(arguments.px4_ulog)
    px4_track = prepare_track(
        "px4_v8",
        px4_source,
        geometry,
        height_column="height_m",
        speed_column="airspeed_m_s",
        roll_column="roll_deg",
    )
    tracks = {**xiangyi_tracks, "px4_v8": px4_track}
    trajectory = {
        "interpretation": (
            "descriptive closed-loop comparison only; controller and guidance "
            "differences are not aerodynamic-model pass/fail evidence"
        ),
        "individual_tracks": {
            name: track_metrics(track, geometry) for name, track in tracks.items()
        },
        "pairwise_progress_aligned": {
            "xiangyi_flight_2_vs_flight_3": pair_track_metrics(
                tracks["xiangyi_flight_2"], tracks["xiangyi_flight_3"]
            ),
            "xiangyi_flight_2_vs_px4_v8": pair_track_metrics(
                tracks["xiangyi_flight_2"], tracks["px4_v8"]
            ),
            "xiangyi_flight_3_vs_px4_v8": pair_track_metrics(
                tracks["xiangyi_flight_3"], tracks["px4_v8"]
            ),
        },
        "route": {
            "reference_latitude_deg": geometry["reference_latitude"],
            "reference_longitude_deg": geometry["reference_longitude"],
            "point_count": len(plan_points),
            "total_length_m": float(geometry["cumulative"][-1]),
        },
    }

    servo_columns = [f"SteeringGear{index}Angle" for index in range(1, 11)]
    data_quality = {
        "source": str(arguments.xiangyi_csv),
        "source_sha256": sha256(arguments.xiangyi_csv),
        "schema": source_metadata,
        "flights": [item.__dict__ for item in flights],
        "formal_complete_flights": [item.name for item in complete],
        "frame_contract_checks": frame_checks,
        "control_command_response_sign_checks": control_sign_checks,
        "actual_servo_feedback_nonzero_samples": {
            name: int(np.count_nonzero(np.abs(source[name].to_numpy()) > 1e-9))
            for name in servo_columns
        },
        "engine_speed_nonzero_samples": int(
            np.count_nonzero(np.abs(source["EngineSpeed"].to_numpy()) > 1e-9)
        ),
        "quality_gate_pass": (
            len(complete) == 2
            and all(item["pass"] for item in frame_checks.values())
            and source_metadata["data_rows"] == 51933
        ),
    }
    if not data_quality["quality_gate_pass"]:
        raise RuntimeError(f"Xiangyi data quality gate failed: {data_quality}")

    classification = classify_coefficients(
        baseline, sensitivity, [item.name for item in complete]
    )
    summary = {
        "sources": {
            "xiangyi_csv": str(arguments.xiangyi_csv),
            "xiangyi_csv_sha256": data_quality["source_sha256"],
            "px4_ulog": str(arguments.px4_ulog),
            "px4_ulog_sha256": sha256(arguments.px4_ulog),
            "mission_plan": str(arguments.plan),
            "mission_plan_sha256": sha256(arguments.plan),
        },
        "method": {
            "body_frame": "Xiangyi RFU converted to PDF/PX4 FRD",
            "specific_force": "9.8 * [Acc_Y, Acc_X, -Acc_Z] m/s2",
            "force_relation": f"F_aero = {MASS_KG:g} * f_specific - F_propulsion",
            "moment_relation": "M_aero = I*omega_dot + omega_cross_(I*omega) - M_propulsion",
            "inertia_relation": (
                "100 kg inertia linearly interpolated from Word table 3 "
                "between 73 kg and 150 kg"
            ),
            "surface_mapping": {
                "delta_a_doc_deg": "FW_Ail",
                "delta_e_doc_deg": "FW_Ele",
                "delta_r_doc_deg": "FW_Rud",
                "delta_c_doc_deg": "Canard",
            },
            "baseline_filter_window_s": 0.5,
            "density": "ISA using NavAltitude",
            "propulsion": "Honghu V8 table, 0.5/0.3 s lag, 3 deg down",
            "no_parameter_identification": True,
        },
        "limitations": [
            "All ten SteeringGear feedback fields are zero; FW_* and Canard are commanded-equivalent angles.",
            "EngineSpeed is zero; propulsion is assumed from the V8 table and sensitivity-bounded by +/-20%.",
            "Without vendor model metadata, the highest possible grade is conditional_consistent.",
            "Trajectory differences describe controller plus guidance plus plant behavior and do not grade aerodynamics.",
        ],
        "baseline": baseline,
        "sensitivity": sensitivity,
        "coefficient_classification": classification,
        "trajectory": trajectory,
        "outputs": {
            "data_quality": str((arguments.output_dir / "xiangyi_data_quality.json").resolve()),
            "timeseries": str(timeseries_path.resolve()),
            "coefficient_plot": str((arguments.output_dir / "coefficient_comparison.png").resolve()),
            "residual_plot": str((arguments.output_dir / "coefficient_residual_vs_state.png").resolve()),
            "trajectory_plot": str((arguments.output_dir / "trajectory_comparison.png").resolve()),
            "trajectory_metrics": str((arguments.output_dir / "trajectory_metrics.json").resolve()),
            "summary": str((arguments.output_dir / "comparison_summary.json").resolve()),
        },
    }

    write_json(arguments.output_dir / "xiangyi_data_quality.json", data_quality)
    write_json(arguments.output_dir / "trajectory_metrics.json", trajectory)
    write_json(arguments.output_dir / "comparison_summary.json", summary)
    plot_coefficients(timeseries, arguments.output_dir / "coefficient_comparison.png")
    plot_residuals(timeseries, arguments.output_dir / "coefficient_residual_vs_state.png")
    plot_trajectory(tracks, geometry, arguments.output_dir / "trajectory_comparison.png")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xiangyi-csv", type=Path, default=DEFAULT_XIANGYI)
    parser.add_argument("--px4-ulog", type=Path, default=DEFAULT_ULOG)
    parser.add_argument("--plan", type=Path, default=DEFAULT_PLAN)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    for name in ("xiangyi_csv", "px4_ulog", "plan"):
        path = getattr(arguments, name).expanduser().resolve()
        setattr(arguments, name, path)
        if not path.exists():
            parser.error(f"{name.replace('_', '-')} does not exist: {path}")
    arguments.output_dir = arguments.output_dir.expanduser().resolve()
    summary = analyze(arguments)
    print(json.dumps(json_safe({
        "quality_gate_pass": True,
        "complete_flights": list(summary["baseline"]),
        "coefficient_classification": summary["coefficient_classification"],
        "outputs": summary["outputs"],
    }), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
