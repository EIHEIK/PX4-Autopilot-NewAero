#!/usr/bin/env python3
"""Run the pinned CLI-Anything/Windows-FreeCAD bridge smoke test."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import tempfile
from pathlib import Path


PIN = "39634a640cf20bc603b4faae4d31069c44821a9a"
INSTALL_ROOT = Path.home() / ".local/share/cli-anything-freecad" / PIN
CLI = INSTALL_ROOT / ".venv/bin/cli-anything-freecad"
PYTHON = INSTALL_ROOT / ".venv/bin/python"
DEFAULT_FREECAD = Path("/mnt/d/Program Files/FreeCAD 1.1/bin/freecadcmd.exe")
ROOT = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT = ROOT / "build/honghu_cad_audit/cli_smoke"


def windows_path(path: Path) -> str:
    return subprocess.run(
        ["wslpath", "-w", str(path.resolve())],
        check=True,
        text=True,
        capture_output=True,
    ).stdout.strip()


def run(command, env=None):
    process = subprocess.run(command, text=True, capture_output=True, env=env)
    if process.returncode:
        raise RuntimeError(
            f"command failed ({process.returncode}): {' '.join(map(str, command))}\n"
            f"stdout={process.stdout}\nstderr={process.stderr}"
        )
    return process


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--freecad-cmd", type=Path, default=DEFAULT_FREECAD)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if not CLI.is_file() or not PYTHON.is_file():
        raise FileNotFoundError(f"Pinned CLI-Anything harness is not installed at {INSTALL_ROOT}")
    if not args.freecad_cmd.is_file():
        raise FileNotFoundError(args.freecad_cmd)

    env = os.environ.copy()
    env["FREECAD_PATH"] = str(args.freecad_cmd.resolve())
    version = run([
        str(PYTHON), "-c",
        "from cli_anything.freecad.utils.freecad_backend import get_version; print(get_version())",
    ], env=env).stdout.strip()

    project = output / "smoke_project.json"
    step = output / "smoke_geometry.step"
    bbox_json = output / "smoke_bbox.json"
    for path in (project, step, bbox_json):
        if path.exists():
            path.unlink()

    command_logs = []
    commands = [
        [str(CLI), "--json", "document", "new", "--name", "BridgeSmoke", "--units", "mm", "-o", str(project)],
        [str(CLI), "--json", "-p", str(project), "part", "add", "box", "-n", "AuditBox", "-P", "length=20", "-P", "width=15", "-P", "height=5"],
        [str(CLI), "--json", "-p", str(project), "part", "add", "cylinder", "-n", "AuditCylinder", "-P", "radius=3", "-P", "height=10", "-pos", "30,0,0"],
        [str(CLI), "--json", "-p", str(project), "export", "render", str(step), "--preset", "step", "--overwrite"],
    ]
    for command in commands:
        process = run(command, env=env)
        command_logs.append({"command": command, "stdout": process.stdout.strip()})

    with tempfile.TemporaryDirectory(prefix="freecad_cli_smoke_") as temp_dir:
        macro = Path(temp_dir) / "verify.py"
        macro.write_text(
            "import json, Part\n"
            f"shape=Part.read({windows_path(step)!r})\n"
            "items=[]\n"
            "for solid in shape.Solids:\n"
            "    b=solid.BoundBox\n"
            "    items.append({'volume_mm3':solid.Volume,'dimensions_mm':[b.XLength,b.YLength,b.ZLength]})\n"
            "items.sort(key=lambda item:item['volume_mm3'], reverse=True)\n"
            f"open({windows_path(bbox_json)!r},'w',encoding='utf-8').write(json.dumps(items,indent=2))\n",
            encoding="utf-8",
        )
        process = run([str(args.freecad_cmd), windows_path(macro)])

    solids = json.loads(bbox_json.read_text(encoding="utf-8"))
    expected = ([20.0, 15.0, 5.0], [6.0, 6.0, 10.0])
    if len(solids) != 2:
        raise RuntimeError(f"Expected 2 solids, got {len(solids)}: {solids}")
    errors = [
        max(abs(actual - target) for actual, target in zip(solid["dimensions_mm"], target_dims))
        for solid, target_dims in zip(solids, expected)
    ]
    report = {
        "ok": max(errors) <= 0.1,
        "pinned_commit": PIN,
        "freecad_version": version,
        "freecad_cmd": str(args.freecad_cmd),
        "skill_path": str(Path("/mnt/c/Users/fly/.codex/skills/cli-anything-freecad/SKILL.md")),
        "bridge_patch": str(ROOT / "Tools/honghu/cad_audit/patches/cli-anything-freecad-wsl.patch"),
        "exported_step": str(step),
        "solids": solids,
        "dimension_max_errors_mm": errors,
        "tolerance_mm": 0.1,
        "commands": command_logs,
        "freecad_stdout_tail": process.stdout[-1000:],
        "freecad_stderr_tail": process.stderr[-1000:],
    }
    (output / "toolchain_report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
