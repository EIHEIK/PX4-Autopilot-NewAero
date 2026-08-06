#!/usr/bin/env python3
"""Audit Honghu landing-gear STEP geometry against the V8 SDF baseline."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import subprocess
import tempfile
from pathlib import Path


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
DEFAULT_CONFIG = HERE / "cad_audit_config.yaml"
DEFAULT_OUTPUT = ROOT / "build" / "honghu_cad_audit"


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def to_windows(path: Path) -> str:
    result = subprocess.run(
        ["wslpath", "-w", str(path.resolve())],
        check=True,
        text=True,
        capture_output=True,
    )
    return result.stdout.strip()


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def bbox_measurement(wheel: dict, kind: str) -> dict:
    box = wheel["bbox_mm"]
    if kind == "main":
        width = box["x_length"]
        diameters = (box["y_length"], box["z_length"])
    else:
        width = box["x_length"]
        diameters = (box["y_length"], box["z_length"])
    diameter = sum(diameters) / 2.0
    residual = abs(diameters[0] - diameters[1]) / 2.0
    return {
        "center_step_mm": box["center"],
        "diameter_mm": diameter,
        "width_mm": width,
        "cylinder_fit_residual_mm": residual,
        "bbox_mm": box,
    }


def derive_model_metrics(model: dict) -> dict:
    wheels = model["wheels"]
    left = bbox_measurement(wheels["left_main_wheel"], "main")
    right = bbox_measurement(wheels["right_main_wheel"], "main")
    nose = bbox_measurement(wheels["nose_wheel"], "nose")
    lc, rc, nc = left["center_step_mm"], right["center_step_mm"], nose["center_step_mm"]
    main_mid = [(lc[i] + rc[i]) / 2.0 for i in range(3)]
    return {
        "track_mm": abs(lc[0] - rc[0]),
        "wheelbase_mm": abs(nc[1] - main_mid[1]),
        "main_center_step_mm": main_mid,
        "nose_center_step_mm": nc,
        "left_main": left,
        "right_main": right,
        "nose": nose,
        "main_diameter_mm": (left["diameter_mm"] + right["diameter_mm"]) / 2.0,
        "main_width_mm": (left["width_mm"] + right["width_mm"]) / 2.0,
        "main_fit_residual_mm": max(
            left["cylinder_fit_residual_mm"], right["cylinder_fit_residual_mm"]
        ),
        "nose_fit_residual_mm": nose["cylinder_fit_residual_mm"],
    }


def mean(values):
    return sum(values) / len(values)


def derive_candidate(metrics: list[dict], config: dict) -> dict:
    align = config["candidate_alignment"]
    track_m = mean([m["track_mm"] for m in metrics]) / 1000.0
    wheelbase_m = mean([m["wheelbase_mm"] for m in metrics]) / 1000.0
    main_radius_m = mean([m["main_diameter_mm"] for m in metrics]) / 2000.0
    main_width_m = mean([m["main_width_mm"] for m in metrics]) / 1000.0
    nose_radius_m = mean([m["nose"]["diameter_mm"] for m in metrics]) / 2000.0
    nose_width_m = mean([m["nose"]["width_mm"] for m in metrics]) / 1000.0
    main_x = align["main_axle_x_m"]
    ground_z = align["ground_contact_z_m"]
    return {
        "status": config["status"],
        "alignment_policy": align["policy"],
        "track_m": track_m,
        "wheelbase_m": wheelbase_m,
        "main_wheel_radius_m": main_radius_m,
        "main_wheel_width_m": main_width_m,
        "nose_wheel_radius_m": nose_radius_m,
        "nose_wheel_width_m": nose_width_m,
        "left_main_center_m": [main_x, track_m / 2.0, ground_z + main_radius_m],
        "right_main_center_m": [main_x, -track_m / 2.0, ground_z + main_radius_m],
        "nose_center_m": [
            main_x + wheelbase_m,
            align["nose_centerline_y_m"],
            ground_z + nose_radius_m,
        ],
        "ground_contact_z_m": ground_z,
        "main_strut_body_anchors_m": align["main_strut_body_anchors_m"],
        "nose_strut_body_anchor_m": align["nose_strut_body_anchor_m"],
        "strut_radius_m": align["strut_radius_m"],
    }


def baseline_metrics(config: dict) -> dict:
    base = config["v8_baseline"]
    left = base["left_main_center_m"]
    right = base["right_main_center_m"]
    nose = base["nose_center_m"]
    return {
        "track_m": abs(left[1] - right[1]),
        "wheelbase_m": abs(nose[0] - 0.5 * (left[0] + right[0])),
        "main_wheel_radius_m": base["main_wheel_radius_m"],
        "main_wheel_width_m": base["main_wheel_width_m"],
        "nose_wheel_radius_m": base["nose_wheel_radius_m"],
        "nose_wheel_width_m": base["nose_wheel_width_m"],
    }


def comparison_rows(step_metrics: list[dict], candidate: dict, baseline: dict, tolerance: float):
    definitions = [
        ("track", "mm", "track_mm", "track_m"),
        ("wheelbase", "mm", "wheelbase_mm", "wheelbase_m"),
        ("main_wheel_diameter", "mm", "main_diameter_mm", "main_wheel_radius_m"),
        ("main_wheel_width", "mm", "main_width_mm", "main_wheel_width_m"),
        ("nose_wheel_diameter", "mm", ("nose", "diameter_mm"), "nose_wheel_radius_m"),
        ("nose_wheel_width", "mm", ("nose", "width_mm"), "nose_wheel_width_m"),
    ]
    rows = []
    for name, unit, step_key, candidate_key in definitions:
        step_values = []
        for metrics in step_metrics:
            value = metrics
            for key in step_key if isinstance(step_key, tuple) else (step_key,):
                value = value[key]
            step_values.append(value)
        if "diameter" in name:
            baseline_value = 2.0 * baseline[candidate_key] * 1000.0
            candidate_value = 2.0 * candidate[candidate_key] * 1000.0
        else:
            baseline_value = baseline[candidate_key] * 1000.0
            candidate_value = candidate[candidate_key] * 1000.0
        delta = max(step_values) - min(step_values)
        rows.append({
            "metric": name,
            "unit": unit,
            "step_a": step_values[0],
            "step_b": step_values[1],
            "step_delta": delta,
            "v8_baseline": baseline_value,
            "cad_candidate": candidate_value,
            "candidate_minus_v8": candidate_value - baseline_value,
            "step_agreement": "PASS" if delta <= tolerance else "FAIL",
        })
    return rows


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def circle(cx, cy, radius, color, dash=""):
    dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
    return f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="{radius:.2f}" fill="none" stroke="{color}" stroke-width="2"{dash_attr}/>'


def write_svg(path: Path, candidate: dict, baseline: dict) -> None:
    scale = 150.0
    panels = []
    panels.append('<rect width="1200" height="430" fill="white"/>')
    panels.append('<text x="25" y="30" font-size="20">Honghu V8 landing-gear geometry: blue=V8, red=STEP candidate</text>')
    panels.append('<text x="25" y="65" font-size="17">Side view (x-z)</text>')
    x0, z0 = 210.0, 240.0
    for data, color, dash in ((baseline, "#2468d8", "8 5"), (candidate, "#d8342a", "")):
        dash_attr = ' stroke-dasharray="8 5"' if dash else ""
        main_x = data["left_main_center_m"][0] if "left_main_center_m" in data else -0.291274
        nose_x = data["nose_center_m"][0] if "nose_center_m" in data else main_x + data["wheelbase_m"]
        main_r = data["main_wheel_radius_m"]
        nose_r = data["nose_wheel_radius_m"]
        main_z = data.get("left_main_center_m", [0, 0, -0.5145 + main_r])[2]
        nose_z = data.get("nose_center_m", [0, 0, -0.5145 + nose_r])[2]
        panels.append(circle(x0 + main_x * scale, z0 - main_z * scale, main_r * scale, color, dash))
        panels.append(circle(x0 + nose_x * scale, z0 - nose_z * scale, nose_r * scale, color, dash))
        panels.append(f'<line x1="{x0 + main_x*scale:.2f}" y1="{z0-main_z*scale:.2f}" x2="{x0+nose_x*scale:.2f}" y2="{z0-nose_z*scale:.2f}" stroke="{color}" stroke-width="1"{dash_attr}/>' )
    panels.append('<text x="425" y="65" font-size="17">Front view (y-z)</text>')
    y0, fz0 = 600.0, 240.0
    for data, color, dash in ((baseline, "#2468d8", "8 5"), (candidate, "#d8342a", "")):
        dash_attr = ' stroke-dasharray="8 5"' if dash else ""
        track = data["track_m"]
        radius = data["main_wheel_radius_m"]
        width = data["main_wheel_width_m"]
        center_z = data.get("left_main_center_m", [0, 0, -0.5145 + radius])[2]
        for side in (-1, 1):
            x = y0 + side * track * scale / 2.0
            panels.append(f'<rect x="{x-width*scale/2:.2f}" y="{fz0-(center_z+radius)*scale:.2f}" width="{width*scale:.2f}" height="{2*radius*scale:.2f}" fill="none" stroke="{color}" stroke-width="2"{dash_attr}/>' )
    panels.append('<text x="815" y="65" font-size="17">Top view (x-y)</text>')
    tx0, ty0 = 940.0, 225.0
    for data, color, dash in ((baseline, "#2468d8", "8 5"), (candidate, "#d8342a", "")):
        dash_attr = ' stroke-dasharray="8 5"' if dash else ""
        main_x = data.get("left_main_center_m", [-0.291274])[0]
        nose_x = data.get("nose_center_m", [main_x + data["wheelbase_m"]])[0]
        track = data["track_m"]
        panels.append(f'<line x1="{tx0+main_x*scale:.2f}" y1="{ty0-track*scale/2:.2f}" x2="{tx0+main_x*scale:.2f}" y2="{ty0+track*scale/2:.2f}" stroke="{color}" stroke-width="2"{dash_attr}/>' )
        panels.append(f'<line x1="{tx0+main_x*scale:.2f}" y1="{ty0:.2f}" x2="{tx0+nose_x*scale:.2f}" y2="{ty0:.2f}" stroke="{color}" stroke-width="2"{dash_attr}/>' )
    panels.append(f'<text x="25" y="400" font-size="15">Candidate: track={candidate["track_m"]*1000:.1f} mm, wheelbase={candidate["wheelbase_m"]*1000:.1f} mm, field confirmation pending.</text>')
    path.write_text('<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="430">' + "".join(panels) + "</svg>\n", encoding="utf-8")


def run_freecad(config: dict, output: Path, freecad_cmd: Path, export_selected: bool) -> dict:
    worker = HERE / "freecad_hhy_worker.py"
    raw_output = output / "freecad_raw_measurements.json"
    selected_step = output / "landing_gear_selection.step" if export_selected else None
    runtime = dict(config)
    runtime["step_models"] = [
        {
            **model,
            "path_original": model["path"],
            "path_windows": to_windows(Path(model["path"])),
        }
        for model in config["step_models"]
    ]
    with tempfile.TemporaryDirectory(prefix="honghu_cad_audit_") as temp_dir:
        temp = Path(temp_dir)
        runtime_path = temp / "runtime_config.json"
        macro_path = temp / "run_worker.py"
        write_json(runtime_path, runtime)
        globals_map = {
            "AUDIT_RUNTIME_CONFIG": to_windows(runtime_path),
            "AUDIT_RAW_OUTPUT": to_windows(raw_output),
            "AUDIT_SELECTED_STEP": to_windows(selected_step) if selected_step else None,
        }
        macro = (
            "import runpy\n"
            f"runpy.run_path({to_windows(worker)!r}, init_globals={globals_map!r})\n"
        )
        macro_path.write_text(macro, encoding="utf-8")
        process = subprocess.run(
            [str(freecad_cmd), to_windows(macro_path)],
            text=True,
            capture_output=True,
            timeout=1800,
        )
    if not raw_output.exists():
        raise RuntimeError(
            "FreeCAD did not produce the audit output. "
            f"returncode={process.returncode}\nstdout={process.stdout}\nstderr={process.stderr}"
        )
    payload = json.loads(raw_output.read_text(encoding="utf-8"))
    if not payload.get("ok"):
        raise RuntimeError(payload.get("traceback", payload.get("error", "FreeCAD audit failed")))
    payload["process"] = {
        "returncode": process.returncode,
        "stdout_tail": process.stdout[-2000:],
        "stderr_tail": process.stderr[-2000:],
    }
    return payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--freecad-cmd", type=Path)
    parser.add_argument("--no-step-export", action="store_true")
    args = parser.parse_args()

    config = load_config(args.config)
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    freecad_cmd = (args.freecad_cmd or Path(config["freecad_cmd_wsl"])).resolve()
    if not freecad_cmd.is_file():
        raise FileNotFoundError(f"FreeCADCmd not found: {freecad_cmd}")
    for model in config["step_models"]:
        if not Path(model["path"]).is_file():
            raise FileNotFoundError(f"STEP model not found: {model['path']}")

    raw = run_freecad(config, output, freecad_cmd, not args.no_step_export)
    metrics = [derive_model_metrics(model) for model in raw["models"]]
    candidate = derive_candidate(metrics, config)
    baseline = baseline_metrics(config)
    rows = comparison_rows(
        metrics, candidate, baseline, config["tolerances"]["step_agreement_mm"]
    )
    max_fit = max(
        max(m["main_fit_residual_mm"], m["nose_fit_residual_mm"])
        for m in metrics
    )
    summary = {
        "schema_version": 1,
        "status": config["status"],
        "freecad_version": raw["freecad_version"],
        "source_models": raw["models"],
        "step_metrics": metrics,
        "candidate": candidate,
        "v8_baseline": baseline,
        "comparison": rows,
        "quality_gates": {
            "step_agreement": all(row["step_agreement"] == "PASS" for row in rows),
            "max_cylinder_fit_residual_mm": max_fit,
            "wheel_fit_pass": max_fit <= config["tolerances"]["wheel_fit_residual_mm"],
            "field_measurement_pass": None,
            "field_measurement_note": "No field measurements supplied; CAD values remain candidates.",
        },
        "coordinate_note": (
            "STEP X/Y/Z are interpreted as lateral/longitudinal/vertical. "
            "Absolute Gazebo placement preserves the V8 main-axle x and ground plane; "
            "only relative STEP dimensions are promoted before datum registration."
        ),
    }
    write_json(output / "cad_measurements.json", summary)
    write_csv(output / "v8_vs_step.csv", rows)
    write_svg(output / "gear_comparison.svg", candidate, {**baseline, **config["v8_baseline"]})
    print(json.dumps({
        "ok": True,
        "output_dir": str(output),
        "candidate": candidate,
        "quality_gates": summary["quality_gates"],
    }, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
