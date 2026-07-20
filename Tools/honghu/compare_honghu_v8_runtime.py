#!/usr/bin/env python3
"""Compare one Gazebo Honghu V8 diagnostic frame with the offline model."""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path

from honghu_v8_aero_model import HonghuV8AeroModel, joint_angles_to_document_deflections


NAMES = ("CL", "CD", "CY", "Cl", "Cm", "Cn")
SURFACES = ("aileron", "elevator", "rudder", "canard")


def read_frame(topic: str) -> list[float]:
    process = subprocess.run(
        ["gz", "topic", "-e", "-n", "1", "--json-output", "-t", topic],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    message = json.loads(process.stdout.strip())
    data = message["data"]
    if len(data) != 76:
        raise RuntimeError(f"expected 76 aero_state values, got {len(data)}")
    return data


def compare(data: list[float], tolerance: float) -> dict:
    speed, alpha, beta = data[0:3]
    alpha_dot, beta_dot = data[4:6]
    p, q, r = data[6:9]
    theta = data[15:23]
    runtime_delta = data[23:27]
    calculated_delta = joint_angles_to_document_deflections(theta)
    model = HonghuV8AeroModel()
    expected, controls = model.coefficients(
        alpha, beta, speed,
        delta_a_deg=runtime_delta[0], delta_e_deg=runtime_delta[1],
        delta_r_deg=runtime_delta[2], delta_c_deg=runtime_delta[3],
        p_rad_s=p, q_rad_s=q, r_rad_s=r,
        alpha_dot_rad_s=alpha_dot, beta_dot_rad_s=beta_dot,
    )

    errors = {}
    for index, name in enumerate(NAMES):
        errors[f"total_{name}"] = data[9 + index] - getattr(expected, name)
    for surface_index, surface in enumerate(SURFACES):
        offset = 27 + surface_index * 6
        for coefficient_index, name in enumerate(NAMES):
            errors[f"{surface}_{name}"] = (
                data[offset + coefficient_index] - getattr(controls[surface], name)
            )
    for index, name in enumerate(("delta_a", "delta_e", "delta_r", "delta_c")):
        errors[name] = runtime_delta[index] - calculated_delta[index]

    maximum = max(abs(value) for value in errors.values())
    if maximum > tolerance:
        worst = max(errors, key=lambda key: abs(errors[key]))
        raise AssertionError(
            f"runtime/offline mismatch {worst}={errors[worst]:+.12g}, "
            f"maximum={maximum:.12g}, tolerance={tolerance:.12g}"
        )
    return {
        "status": "PASS",
        "speed_mps": speed,
        "alpha_deg": alpha,
        "beta_deg": beta,
        "delta_doc_deg": dict(zip(("a", "e", "r", "c"), runtime_delta)),
        "maximum_absolute_error": maximum,
        "tolerance": tolerance,
        "flags": int(data[75]),
        "errors": errors,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--topic", default="/model/honghu_runtime/honghu_v8/aero_state",
        help="gz.msgs.Double_V diagnostic topic",
    )
    parser.add_argument("--tolerance", type=float, default=2e-10)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    report = compare(read_frame(args.topic), args.tolerance)
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(
        "Honghu V8 Gazebo/offline aero comparison: PASS\n"
        f"  V={report['speed_mps']:.6f} m/s, alpha={report['alpha_deg']:.6f} deg, "
        f"beta={report['beta_deg']:.6f} deg\n"
        f"  delta_doc={report['delta_doc_deg']}\n"
        f"  maximum coefficient/control/mapping error={report['maximum_absolute_error']:.3e}"
    )


if __name__ == "__main__":
    main()
