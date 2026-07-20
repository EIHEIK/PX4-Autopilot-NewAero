#!/usr/bin/env python3
"""Compare Honghu V5 aero-force CSV output with table-formula reconstruction.

The HonghuAeroTable plugin writes body/world six-component aerodynamic data to
``honghu_v5_aero_forces.csv`` when ``<force_log>true</force_log>`` is enabled.
This script checks whether the logged body-frame force/moment components match
the coefficients and reference quantities recorded in the same CSV.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


DEFAULT_LOG = Path(
    "/home/fly/PX4-Autopilot-canard-2026.6.2/"
    "build/px4_sitl_default/rootfs/honghu_v5_aero_forces.csv"
)

AREA = 2.42
SPAN = 3.96
MAC = 0.62


def vec_norm(values: list[float]) -> float:
    return math.sqrt(sum(v * v for v in values))


def reconstruct_body_force(row: dict[str, str]) -> tuple[list[float], list[float]]:
    alpha = math.radians(float(row["alpha_deg"]))
    beta = math.radians(float(row["beta_deg"]))
    qbar = float(row["qbar_pa"])
    cl = float(row["CL"])
    cd = float(row["CD"])
    cy = float(row["CY"])
    roll = float(row["Cl"])
    pitch = float(row["Cm"])
    yaw = float(row["Cn"])

    # Reconstruct the same body-frame velocity direction used by the plugin:
    # alpha = atan2(-z, x), beta = atan2(y, x).
    wind_x = [1.0, math.tan(beta), -math.tan(alpha)]
    n = vec_norm(wind_x)
    wind_x = [v / n for v in wind_x]

    upward = [0.0, 0.0, 1.0]
    dot = sum(wind_x[i] * upward[i] for i in range(3))
    lift_axis = [upward[i] - dot * wind_x[i] for i in range(3)]
    n = vec_norm(lift_axis)
    if n < 1e-9:
        lift_axis = upward[:]
    else:
        lift_axis = [v / n for v in lift_axis]

    # side = lift_axis x wind_x, matching HonghuAeroTable.cpp.
    side_axis = [
        lift_axis[1] * wind_x[2] - lift_axis[2] * wind_x[1],
        lift_axis[2] * wind_x[0] - lift_axis[0] * wind_x[2],
        lift_axis[0] * wind_x[1] - lift_axis[1] * wind_x[0],
    ]
    n = vec_norm(side_axis)
    side_axis = [v / n for v in side_axis]

    force = [
        qbar * AREA * (cl * lift_axis[i] - cd * wind_x[i] + cy * side_axis[i])
        for i in range(3)
    ]

    # In the SDF z-up frame the plugin maps body moment axes as:
    # roll +X, pitch -Y, yaw -Z.
    moment = [
        roll * qbar * AREA * SPAN,
        -pitch * qbar * AREA * MAC,
        -yaw * qbar * AREA * SPAN,
    ]

    return force, moment


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="?", type=Path, default=DEFAULT_LOG)
    args = parser.parse_args()

    if not args.csv.exists():
        raise SystemExit(f"CSV not found: {args.csv}")

    rows = []
    with args.csv.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    if not rows:
        raise SystemExit(f"CSV is empty: {args.csv}")

    max_force_err = 0.0
    max_moment_err = 0.0
    sum_force_err2 = 0.0
    sum_moment_err2 = 0.0

    for row in rows:
        force_expected, moment_expected = reconstruct_body_force(row)
        force_logged = [float(row[f"force_body_{axis}_N"]) for axis in "xyz"]
        moment_logged = [float(row[f"moment_body_{axis}_Nm"]) for axis in "xyz"]

        force_err = vec_norm([force_logged[i] - force_expected[i] for i in range(3)])
        moment_err = vec_norm([moment_logged[i] - moment_expected[i] for i in range(3)])
        max_force_err = max(max_force_err, force_err)
        max_moment_err = max(max_moment_err, moment_err)
        sum_force_err2 += force_err * force_err
        sum_moment_err2 += moment_err * moment_err

    first = rows[0]
    last = rows[-1]
    print(f"CSV: {args.csv}")
    print(f"rows: {len(rows)}")
    print(f"time: {float(first['time_s']):.3f} -> {float(last['time_s']):.3f} s")
    print(f"airspeed: {float(first['airspeed_m_s']):.3f} -> {float(last['airspeed_m_s']):.3f} m/s")
    print(f"max force residual:  {max_force_err:.6g} N")
    print(f"rms force residual:  {(sum_force_err2 / len(rows)) ** 0.5:.6g} N")
    print(f"max moment residual: {max_moment_err:.6g} N*m")
    print(f"rms moment residual: {(sum_moment_err2 / len(rows)) ** 0.5:.6g} N*m")
    print("latest body six-component:")
    print(
        "  F_body[N] = "
        f"({float(last['force_body_x_N']):.3f}, "
        f"{float(last['force_body_y_N']):.3f}, "
        f"{float(last['force_body_z_N']):.3f})"
    )
    print(
        "  M_body[Nm] = "
        f"({float(last['moment_body_x_Nm']):.3f}, "
        f"{float(last['moment_body_y_Nm']):.3f}, "
        f"{float(last['moment_body_z_Nm']):.3f})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
