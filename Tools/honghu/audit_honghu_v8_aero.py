#!/usr/bin/env python3
"""Offline aerodynamic acceptance and trim report for Honghu Wing V8."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from honghu_v8_aero_model import (
    DEFAULT_TABLE_DIR,
    Coefficients,
    HonghuV8AeroModel,
    joint_angles_to_document_deflections,
)


COEFFICIENT_NAMES = ("CL", "CD", "CY", "Cl", "Cm", "Cn")
PLAN_REFERENCES = {
    35.0: (6.47, 4.13),
    40.0: (3.99, 1.28),
    45.0: (2.35, 0.28),
}


class Audit:
    def __init__(self) -> None:
        self.checks = 0

    def require(self, condition: bool, message: str) -> None:
        self.checks += 1
        if not condition:
            raise AssertionError(message)

    def close(self, actual: float, expected: float, tolerance: float, message: str) -> None:
        self.require(math.isclose(actual, expected, rel_tol=0.0, abs_tol=tolerance),
                     f"{message}: expected {expected:.12g}, got {actual:.12g}")


def coefficient_delta(a: Coefficients, b: Coefficients) -> float:
    return max(abs(getattr(a, name) - getattr(b, name)) for name in COEFFICIENT_NAMES)


def audit_model(model: HonghuV8AeroModel) -> dict:
    audit = Audit()

    # Every published CSV node must reproduce its source value exactly.
    for grid in model.all_grids():
        for i, row in enumerate(grid.rows):
            for j, column in enumerate(grid.columns):
                audit.close(grid.interpolate(row, column), grid.values[i][j], 2e-12,
                            f"node reproduction {grid.path.name}({row},{column})")

    # Static symmetry is imposed once in the core: longitudinal even in beta,
    # lateral odd.  This catches duplicate or missing sign flips.
    for alpha in (-8.0, 0.0, 8.0, 16.0, 24.0):
        positive = model.static_coefficients(alpha, 7.0)
        negative = model.static_coefficients(alpha, -7.0)
        for name in ("CL", "CD", "Cm"):
            audit.close(getattr(positive, name), getattr(negative, name), 2e-12,
                        f"{name} beta-even at alpha={alpha}")
        for name in ("CY", "Cl", "Cn"):
            audit.close(getattr(positive, name), -getattr(negative, name), 2e-12,
                        f"{name} beta-odd at alpha={alpha}")

    # Zero virtual deflection must have exactly zero control contribution even
    # though the local derivative at zero is generally non-zero.
    zero = model.control_contributions(0.0, 0.0)
    for surface, contribution in zero.items():
        for name in COEFFICIENT_NAMES:
            audit.close(getattr(contribution, name), 0.0, 0.0,
                        f"zero-deflection {surface} {name}")

    # Primary FRD moment sign and the one-time FRD -> Gazebo FLU conversion.
    sign_cases = (
        ("aileron", "Cl", "delta_a_deg", "x", 1.0),
        ("elevator", "Cm", "delta_e_deg", "y", -1.0),
        ("rudder", "Cn", "delta_r_deg", "z", -1.0),
        ("canard", "Cm", "delta_c_deg", "y", -1.0),
    )
    axis_index = {"x": 0, "y": 1, "z": 2}
    signs = {}
    neutral = model.evaluate(0.0, 0.0, 30.0)
    for surface, coefficient, argument, gz_axis, gz_sign in sign_cases:
        positive = model.evaluate(0.0, 0.0, 30.0, **{argument: 1.0})
        negative = model.evaluate(0.0, 0.0, 30.0, **{argument: -1.0})
        dp = getattr(positive.controls[surface], coefficient)
        dn = getattr(negative.controls[surface], coefficient)
        audit.require(dp > 0.0, f"positive {surface} must produce positive FRD {coefficient}")
        audit.require(dn < 0.0, f"negative {surface} must produce negative FRD {coefficient}")
        index = axis_index[gz_axis]
        mp = positive.moment_gz_flu_nm[index] - neutral.moment_gz_flu_nm[index]
        mn = negative.moment_gz_flu_nm[index] - neutral.moment_gz_flu_nm[index]
        audit.require(mp * gz_sign > 0.0 and mn * gz_sign < 0.0,
                      f"{surface} Gazebo {gz_axis} moment conversion failed")
        signs[surface] = {"positive_control_FRD_delta": dp, "positive_control_GZ_moment_Nm": mp}

    audit.require(joint_angles_to_document_deflections((-1, 1, 1, 1, 1, 1, 1, 1)) == (1, 1, 1, 1),
                  "joint angle to document deflection map failed")

    # Value continuity at every piecewise boundary.  Derivative continuity is
    # not required because the PDF itself is a sparse piecewise-linear table.
    continuity = {}
    eps = 1e-7
    for boundary in (-12.0, 20.0):
        below = model.static_coefficients(boundary - eps, 5.0)
        above = model.static_coefficients(boundary + eps, 5.0)
        jump = coefficient_delta(below, above)
        audit.require(jump < 2e-6, f"static/Viterna discontinuity at alpha={boundary}: {jump}")
        continuity[f"alpha_{boundary:g}"] = jump
    below = model.static_coefficients(4.0, 16.0 - eps)
    above = model.static_coefficients(4.0, 16.0 + eps)
    jump = coefficient_delta(below, above)
    audit.require(jump < 2e-8, f"beta clamp discontinuity: {jump}")
    continuity["beta_16"] = jump
    for boundary in (12.0, 16.0):
        left = model.control_contributions(boundary - 4.0 - eps, 0.0, delta_c_deg=4.0)["canard"]
        right = model.control_contributions(boundary - 4.0 + eps, 0.0, delta_c_deg=4.0)["canard"]
        jump = coefficient_delta(left, right)
        audit.require(jump < 2e-8, f"canard stall fade discontinuity at effective alpha={boundary}: {jump}")
        continuity[f"canard_effective_alpha_{boundary:g}"] = jump

    # Damping signs: positive p/q/r must oppose their principal moments.
    roll, _ = model.coefficients(0, 0, 30, p_rad_s=0.1)
    pitch, _ = model.coefficients(0, 0, 30, q_rad_s=0.1)
    yaw, _ = model.coefficients(0, 0, 30, r_rad_s=0.1)
    baseline, _ = model.coefficients(0, 0, 30)
    audit.require(roll.Cl - baseline.Cl < 0.0, "Clp must damp positive roll rate")
    audit.require(pitch.Cm - baseline.Cm < 0.0, "Cmq must damp positive pitch rate")
    audit.require(yaw.Cn - baseline.Cn < 0.0, "Cnr must damp positive yaw rate")

    trims = []
    reference_residuals = []
    for speed, (reference_alpha, reference_elevator) in PLAN_REFERENCES.items():
        trim = model.solve_longitudinal_trim(speed)
        trims.append(trim)
        audit.require(-10.0 <= trim["elevator_deg"] <= 20.0,
                      f"trim elevator outside directly tabulated range at {speed:g} m/s")
        result = model.coefficients(
            reference_alpha, 0.0, speed,
            delta_e_deg=reference_elevator, delta_c_deg=4.0,
        )[0]
        reference_residuals.append({
            "speed_mps": speed,
            "reference_alpha_deg": reference_alpha,
            "reference_elevator_deg": reference_elevator,
            "CL_minus_trim_target": result.CL - trim["target_CL"],
            "Cm": result.Cm,
            "status": "engineering_reference_only_not_a_PDF_acceptance_target",
        })

    return {
        "status": "PASS",
        "checks": audit.checks,
        "table_dir": str(model.table_dir),
        "signs": signs,
        "continuity_max_jumps": continuity,
        "trim_canard_4_deg": trims,
        "previous_plan_reference_residuals": reference_residuals,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--table-dir", type=Path, default=DEFAULT_TABLE_DIR)
    parser.add_argument("--json", type=Path, help="also write the complete audit report as JSON")
    args = parser.parse_args()
    report = audit_model(HonghuV8AeroModel(args.table_dir))
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    print(f"Honghu V8 aerodynamic audit: {report['status']} ({report['checks']} checks)")
    print("  PDF/PX4 FRD -> Gazebo FLU control moment signs: PASS")
    print("  table nodes, beta symmetry, zero controls, boundaries and damping: PASS")
    print("  canard +4 deg longitudinal trims:")
    for trim in report["trim_canard_4_deg"]:
        print(
            f"    {trim['speed_mps']:g} m/s: alpha={trim['alpha_deg']:.4f} deg, "
            f"elevator(up+)={trim['elevator_deg']:.4f} deg, "
            f"CL={trim['CL']:.6f}, Cm={trim['Cm']:+.3e}"
        )
    print("  prior plan values are retained only as non-PDF engineering references; residuals are in JSON")


if __name__ == "__main__":
    main()
