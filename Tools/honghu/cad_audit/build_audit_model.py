#!/usr/bin/env python3
"""Build the isolated Honghu V8 CAD landing-gear audit SDF."""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
BASELINE_SDF = ROOT / "simulation_models/models/honghu_wing_150kg_v8/model.sdf"
AUDIT_JSON = ROOT / "build/honghu_cad_audit/cad_measurements.json"
OUTPUT_SDF = ROOT / "simulation_models/models/honghu_wing_150kg_v8_cad_audit/model.sdf"
MODEL_NAME = "honghu_wing_150kg_v8_cad_audit"
BASE_MODEL = "honghu_wing_150kg_v8"


def set_text(parent, tag, text):
    element = parent.find(tag)
    if element is None:
        element = ET.SubElement(parent, tag)
    element.text = str(text)
    return element


def find_named(model, tag, name):
    element = model.find(f"{tag}[@name='{name}']")
    if element is None:
        raise RuntimeError(f"Missing {tag} named {name}")
    return element


def cylinder_visual(visual, radius, length, color):
    for child in list(visual):
        visual.remove(child)
    set_text(visual, "pose", "0 0 0 1.57079632679 0 0")
    geometry = ET.SubElement(visual, "geometry")
    cylinder = ET.SubElement(geometry, "cylinder")
    set_text(cylinder, "radius", f"{radius:.9f}")
    set_text(cylinder, "length", f"{length:.9f}")
    material = ET.SubElement(visual, "material")
    set_text(material, "ambient", color)
    set_text(material, "diffuse", color)


def update_collision(collision, radius, length):
    cylinder = collision.find("geometry/cylinder")
    if cylinder is None:
        raise RuntimeError(f"Collision {collision.attrib.get('name')} is not cylindrical")
    set_text(cylinder, "radius", f"{radius:.9f}")
    set_text(cylinder, "length", f"{length:.9f}")


def fmt_pose(values):
    return " ".join(f"{value:.9f}" for value in values)


def parallel_axis(mass, position):
    x, y, z = position
    return [
        [mass * (y * y + z * z), -mass * x * y, -mass * x * z],
        [-mass * x * y, mass * (x * x + z * z), -mass * y * z],
        [-mass * x * z, -mass * y * z, mass * (x * x + y * y)],
    ]


def matrix_add(*matrices):
    return [
        [sum(matrix[i][j] for matrix in matrices) for j in range(3)]
        for i in range(3)
    ]


def matrix_sub(left, right):
    return [[left[i][j] - right[i][j] for j in range(3)] for i in range(3)]


def inertia_matrix(inertial):
    inertia = inertial.find("inertia")
    if inertia is None:
        raise RuntimeError("Missing inertia tensor")
    ixx = float(inertia.findtext("ixx"))
    iyy = float(inertia.findtext("iyy"))
    izz = float(inertia.findtext("izz"))
    ixy = float(inertia.findtext("ixy"))
    ixz = float(inertia.findtext("ixz"))
    iyz = float(inertia.findtext("iyz"))
    return [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]]


def set_inertia_matrix(inertial, matrix):
    inertia = inertial.find("inertia")
    if inertia is None:
        raise RuntimeError("Missing inertia tensor")
    values = {
        "ixx": matrix[0][0], "iyy": matrix[1][1], "izz": matrix[2][2],
        "ixy": matrix[0][1], "ixz": matrix[0][2], "iyz": matrix[1][2],
    }
    for tag, value in values.items():
        set_text(inertia, tag, f"{value:.12g}")


def joint_position(model, name):
    joint = find_named(model, "joint", name)
    pose = joint.findtext("pose")
    if not pose:
        raise RuntimeError(f"Joint {name} has no pose")
    return [float(value) for value in pose.split()[:3]]


def link_mass(model, name):
    link = find_named(model, "link", name)
    value = link.findtext("inertial/mass")
    if value is None:
        raise RuntimeError(f"Link {name} has no mass")
    return float(value)


def preserve_baseline_mass_properties(model, candidate):
    """Compensate the residual base inertia for moved wheel-group masses.

    Wheel masses and intrinsic inertias intentionally remain those already
    deducted from the Word/PDF whole-aircraft tensor. Moving their joints must
    therefore be offset in base_link so the candidate keeps the baseline CG and
    complete inertia exactly, rather than quietly changing the flight plant.
    """
    moved_groups = [
        (link_mass(model, "left_main_wheel"),
         joint_position(model, "left_main_wheel_joint"), candidate["left_main_center_m"]),
        (link_mass(model, "right_main_wheel"),
         joint_position(model, "right_main_wheel_joint"), candidate["right_main_center_m"]),
        (link_mass(model, "nose_steering_fork") + link_mass(model, "nose_wheel"),
         joint_position(model, "nose_steering_joint"), candidate["nose_center_m"]),
    ]

    base = find_named(model, "link", "base_link")
    inertial = base.find("inertial")
    if inertial is None:
        raise RuntimeError("base_link has no inertial block")
    base_mass = float(inertial.findtext("mass"))
    pose = [float(value) for value in inertial.findtext("pose").split()]
    old_com = pose[:3]
    first_moment_delta = [
        sum(mass * (new[i] - old[i]) for mass, old, new in moved_groups)
        for i in range(3)
    ]
    new_com = [old_com[i] - first_moment_delta[i] / base_mass for i in range(3)]

    corrected = matrix_add(
        inertia_matrix(inertial),
        parallel_axis(base_mass, old_com),
    )
    corrected = matrix_sub(corrected, parallel_axis(base_mass, new_com))
    for mass, old, new in moved_groups:
        corrected = matrix_add(corrected, parallel_axis(mass, old))
        corrected = matrix_sub(corrected, parallel_axis(mass, new))

    pose[:3] = new_com
    set_text(inertial, "pose", fmt_pose(pose))
    set_inertia_matrix(inertial, corrected)


def add_strut_visual(base_link, name, start, end, radius, color):
    delta = [end[i] - start[i] for i in range(3)]
    length = math.sqrt(sum(value * value for value in delta))
    if length <= 0:
        raise ValueError(f"Zero-length strut: {name}")
    midpoint = [(start[i] + end[i]) / 2.0 for i in range(3)]
    horizontal = math.hypot(delta[0], delta[1])
    pitch = math.atan2(horizontal, delta[2])
    yaw = math.atan2(delta[1], delta[0])
    visual = ET.SubElement(base_link, "visual", {"name": name})
    set_text(visual, "pose", fmt_pose((*midpoint, 0.0, pitch, yaw)))
    geometry = ET.SubElement(visual, "geometry")
    cylinder = ET.SubElement(geometry, "cylinder")
    set_text(cylinder, "radius", f"{radius:.9f}")
    set_text(cylinder, "length", f"{length:.9f}")
    material = ET.SubElement(visual, "material")
    set_text(material, "ambient", color)
    set_text(material, "diffuse", color)


def build(candidate: dict, baseline_sdf: Path, output: Path, model_name: str, asset_model: str) -> None:
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(baseline_sdf, parser=parser)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        raise RuntimeError("Baseline SDF has no model element")
    model.attrib["name"] = model_name

    preserve_baseline_mass_properties(model, candidate)

    for uri in model.findall(".//mesh/uri"):
        if uri.text and uri.text.startswith("meshes/"):
            uri.text = f"model://{asset_model}/{uri.text}"

    base_link = find_named(model, "link", "base_link")
    body_visual = find_named(base_link, "visual", "body_visual")
    set_text(body_visual, "transparency", "0.55")

    wheel_specs = {
        "left": {
            "joint": "left_main_wheel_joint",
            "link": "left_main_wheel",
            "collision": "left_main_wheel_collision",
            "center": candidate["left_main_center_m"],
            "radius": candidate["main_wheel_radius_m"],
            "width": candidate["main_wheel_width_m"],
            "color": "0.85 0.08 0.04 1",
        },
        "right": {
            "joint": "right_main_wheel_joint",
            "link": "right_main_wheel",
            "collision": "right_main_wheel_collision",
            "center": candidate["right_main_center_m"],
            "radius": candidate["main_wheel_radius_m"],
            "width": candidate["main_wheel_width_m"],
            "color": "0.85 0.08 0.04 1",
        },
        "nose": {
            "joint": "nose_steering_joint",
            "link": "nose_wheel",
            "collision": "nose_wheel_collision",
            "center": candidate["nose_center_m"],
            "radius": candidate["nose_wheel_radius_m"],
            "width": candidate["nose_wheel_width_m"],
            "color": "1 0.45 0.02 1",
        },
    }
    for spec in wheel_specs.values():
        joint = find_named(model, "joint", spec["joint"])
        pose = joint.find("pose")
        if pose is None:
            raise RuntimeError(f"Joint {spec['joint']} has no pose")
        pose.text = fmt_pose((*spec["center"], 0.0, 0.0, 0.0))
        link = find_named(model, "link", spec["link"])
        visual = find_named(link, "visual", "wheel_visual")
        cylinder_visual(visual, spec["radius"], spec["width"], spec["color"])
        collision = find_named(link, "collision", spec["collision"])
        update_collision(collision, spec["radius"], spec["width"])

    anchors = candidate["main_strut_body_anchors_m"]
    add_strut_visual(
        base_link, "cad_left_main_strut", anchors["left"],
        candidate["left_main_center_m"], candidate["strut_radius_m"], "0.95 0.75 0.05 1",
    )
    add_strut_visual(
        base_link, "cad_right_main_strut", anchors["right"],
        candidate["right_main_center_m"], candidate["strut_radius_m"], "0.95 0.75 0.05 1",
    )
    add_strut_visual(
        base_link, "cad_nose_strut", candidate["nose_strut_body_anchor_m"],
        candidate["nose_center_m"], candidate["strut_radius_m"], "0.95 0.75 0.05 1",
    )

    metadata = ET.Comment(
        " CAD candidate only: STEP-derived relative dimensions, V8 main-axle/ground anchors, field confirmation pending. "
    )
    model.insert(0, metadata)
    ET.indent(tree, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit-json", type=Path, default=AUDIT_JSON)
    parser.add_argument("--baseline-sdf", type=Path, default=BASELINE_SDF)
    parser.add_argument("--output", type=Path, default=OUTPUT_SDF)
    parser.add_argument("--model-name", default=MODEL_NAME)
    parser.add_argument("--asset-model", default=BASE_MODEL)
    args = parser.parse_args()
    payload = json.loads(args.audit_json.read_text(encoding="utf-8"))
    candidate = payload["candidate"]
    if not candidate["status"].startswith("cad_candidate"):
        raise RuntimeError(f"Unexpected candidate status: {candidate['status']}")
    build(candidate, args.baseline_sdf, args.output, args.model_name, args.asset_model)
    print(json.dumps({
        "ok": True,
        "baseline_sdf": str(args.baseline_sdf),
        "output": str(args.output),
        "model_name": args.model_name,
        "asset_model": args.asset_model,
        "candidate": candidate,
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
