#!/usr/bin/env python3
"""Validate generated 4039 SDF geometry against the CAD audit JSON."""

from __future__ import annotations

import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
DEFAULT_AUDIT = ROOT / "build/honghu_cad_audit/cad_measurements.json"
DEFAULT_SDF = ROOT / "simulation_models/models/honghu_wing_150kg_v8_cad_audit/model.sdf"
DEFAULT_BASELINE_SDF = ROOT / "simulation_models/models/honghu_wing_150kg_v8/model.sdf"
DEFAULT_REPORT = ROOT / "build/honghu_cad_audit/sdf_geometry_validation.json"


def pose_values(model, joint_name):
    pose = model.find(f"joint[@name='{joint_name}']/pose")
    if pose is None or not pose.text:
        raise RuntimeError(f"Missing pose for {joint_name}")
    return [float(value) for value in pose.text.split()[:3]]


def cylinder_values(model, link_name, collision_name):
    cylinder = model.find(
        f"link[@name='{link_name}']/collision[@name='{collision_name}']/geometry/cylinder"
    )
    if cylinder is None:
        raise RuntimeError(f"Missing cylinder {link_name}/{collision_name}")
    return float(cylinder.findtext("radius")), float(cylinder.findtext("length"))


def max_error(actual, expected):
    if isinstance(expected, list):
        return max(abs(a - e) for a, e in zip(actual, expected))
    return abs(actual - expected)


def inertia_matrix(link):
    inertia = link.find("inertial/inertia")
    if inertia is None:
        raise RuntimeError(f"Missing inertia for {link.attrib.get('name')}")
    ixx = float(inertia.findtext("ixx"))
    iyy = float(inertia.findtext("iyy"))
    izz = float(inertia.findtext("izz"))
    ixy = float(inertia.findtext("ixy"))
    ixz = float(inertia.findtext("ixz"))
    iyz = float(inertia.findtext("iyz"))
    return [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]


def parallel_axis(mass, position):
    x, y, z = position
    return [
        [mass * (y * y + z * z), -mass * x * y, -mass * x * z],
        [-mass * x * y, mass * (x * x + z * z), -mass * y * z],
        [-mass * x * z, -mass * y * z, mass * (x * x + y * y)],
    ]


def matrix_add(left, right):
    return [[left[i][j] + right[i][j] for j in range(3)] for i in range(3)]


def moving_mass_signature(model):
    """Return the mass properties affected by relocating the three wheel groups."""
    base = model.find("link[@name='base_link']")
    if base is None:
        raise RuntimeError("Missing base_link")
    base_mass = float(base.findtext("inertial/mass"))
    base_position = [float(value) for value in base.findtext("inertial/pose").split()[:3]]
    first_moment = [base_mass * value for value in base_position]
    inertia_origin = matrix_add(inertia_matrix(base), parallel_axis(base_mass, base_position))

    groups = (
        (("left_main_wheel",), "left_main_wheel_joint"),
        (("right_main_wheel",), "right_main_wheel_joint"),
        (("nose_steering_fork", "nose_wheel"), "nose_steering_joint"),
    )
    for link_names, joint_name in groups:
        group_mass = 0.0
        group_intrinsic = [[0.0] * 3 for _ in range(3)]
        for link_name in link_names:
            link = model.find(f"link[@name='{link_name}']")
            if link is None:
                raise RuntimeError(f"Missing link {link_name}")
            group_mass += float(link.findtext("inertial/mass"))
            group_intrinsic = matrix_add(group_intrinsic, inertia_matrix(link))
        position = pose_values(model, joint_name)
        first_moment = [
            first_moment[i] + group_mass * position[i] for i in range(3)
        ]
        inertia_origin = matrix_add(inertia_origin, group_intrinsic)
        inertia_origin = matrix_add(inertia_origin, parallel_axis(group_mass, position))
    return first_moment, inertia_origin


def maximum_nested_delta(left, right):
    if isinstance(left[0], list):
        return max(abs(left[i][j] - right[i][j]) for i in range(3) for j in range(3))
    return max(abs(left[i] - right[i]) for i in range(3))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit-json", type=Path, default=DEFAULT_AUDIT)
    parser.add_argument("--sdf", type=Path, default=DEFAULT_SDF)
    parser.add_argument("--baseline-sdf", type=Path, default=DEFAULT_BASELINE_SDF)
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--expected-model-name", default="honghu_wing_150kg_v8_cad_audit")
    parser.add_argument("--asset-model", default="honghu_wing_150kg_v8")
    args = parser.parse_args()
    audit = json.loads(args.audit_json.read_text(encoding="utf-8"))
    candidate = audit["candidate"]
    model = ET.parse(args.sdf).getroot().find("model")
    if model is None:
        raise RuntimeError("SDF model missing")

    checks = {
        "left_main_center_m": (
            pose_values(model, "left_main_wheel_joint"), candidate["left_main_center_m"]
        ),
        "right_main_center_m": (
            pose_values(model, "right_main_wheel_joint"), candidate["right_main_center_m"]
        ),
        "nose_center_m": (
            pose_values(model, "nose_steering_joint"), candidate["nose_center_m"]
        ),
    }
    left_radius, left_width = cylinder_values(
        model, "left_main_wheel", "left_main_wheel_collision"
    )
    nose_radius, nose_width = cylinder_values(model, "nose_wheel", "nose_wheel_collision")
    checks.update({
        "main_wheel_radius_m": (left_radius, candidate["main_wheel_radius_m"]),
        "main_wheel_width_m": (left_width, candidate["main_wheel_width_m"]),
        "nose_wheel_radius_m": (nose_radius, candidate["nose_wheel_radius_m"]),
        "nose_wheel_width_m": (nose_width, candidate["nose_wheel_width_m"]),
    })
    results = {
        name: {
            "actual": actual,
            "expected": expected,
            "error_mm": 1000.0 * max_error(actual, expected),
        }
        for name, (actual, expected) in checks.items()
    }
    max_error_mm = max(item["error_mm"] for item in results.values())
    baseline_model = ET.parse(args.baseline_sdf).getroot().find("model")
    if baseline_model is None:
        raise RuntimeError("Baseline SDF model missing")
    baseline_moment, baseline_inertia = moving_mass_signature(baseline_model)
    candidate_moment, candidate_inertia = moving_mass_signature(model)
    first_moment_error = maximum_nested_delta(candidate_moment, baseline_moment)
    inertia_error = maximum_nested_delta(candidate_inertia, baseline_inertia)
    mass_properties_ok = first_moment_error <= 2e-8 and inertia_error <= 1e-8
    mesh_uris = [uri.text for uri in model.findall(".//mesh/uri")]
    report = {
        "ok": (
            model.attrib.get("name") == args.expected_model_name
            and max_error_mm <= 1.0
            and mass_properties_ok
            and all(uri and uri.startswith(f"model://{args.asset_model}/") for uri in mesh_uris)
        ),
        "model_name": model.attrib.get("name"),
        "max_geometry_error_mm": max_error_mm,
        "tolerance_mm": 1.0,
        "checks": results,
        "mesh_uri_count": len(mesh_uris),
        "all_meshes_reference_protected_v8": all(
            uri and uri.startswith(f"model://{args.asset_model}/") for uri in mesh_uris
        ),
        "baseline_mass_property_preservation": {
            "ok": mass_properties_ok,
            "maximum_first_moment_error_kg_m": first_moment_error,
            "maximum_inertia_error_kg_m2": inertia_error,
            "first_moment_tolerance_kg_m": 2e-8,
            "inertia_tolerance_kg_m2": 1e-8,
        },
        "field_measurement_gate": audit["quality_gates"]["field_measurement_pass"],
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
