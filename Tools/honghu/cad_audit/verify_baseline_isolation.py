#!/usr/bin/env python3
"""Snapshot or verify protected Honghu 4028/4038 and shared assets."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
DEFAULT_MANIFEST = ROOT / "build/honghu_cad_audit/protected_baseline_sha256.json"
PROTECTED = [
    Path("ROMFS/px4fmu_common/init.d-posix/airframes/4028_gz_honghu_wing_150kg_v8"),
    Path("ROMFS/px4fmu_common/init.d-posix/airframes/4038_gz_honghu_wing_100kg_v8_xiangyi_test"),
    Path("simulation_models/models/honghu_wing_150kg_v8/model.sdf"),
    Path("simulation_models/models/honghu_wing_150kg_v8/meshes"),
    Path("simulation_models/models/honghu_wing_150kg_v8/aero_tables"),
    Path("simulation_models/models/honghu_wing_150kg_v8/propulsion_tables"),
    Path("simulation_models/models/honghu_wing_100kg_v8_xiangyi_test/model.sdf"),
]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def collect() -> dict:
    files = []
    for relative in PROTECTED:
        target = ROOT / relative
        if target.is_dir():
            files.extend(path for path in target.rglob("*") if path.is_file())
        elif target.is_file():
            files.append(target)
        else:
            raise FileNotFoundError(target)
    return {
        "schema_version": 1,
        "root": str(ROOT),
        "files": {
            str(path.relative_to(ROOT)): sha256(path)
            for path in sorted(set(files))
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("snapshot", "verify"))
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    args = parser.parse_args()
    current = collect()
    if args.mode == "snapshot":
        args.manifest.parent.mkdir(parents=True, exist_ok=True)
        args.manifest.write_text(json.dumps(current, indent=2) + "\n", encoding="utf-8")
        print(f"snapshotted {len(current['files'])} protected files: {args.manifest}")
        return 0
    expected = json.loads(args.manifest.read_text(encoding="utf-8"))
    added = sorted(set(current["files"]) - set(expected["files"]))
    removed = sorted(set(expected["files"]) - set(current["files"]))
    changed = sorted(
        path for path in set(current["files"]) & set(expected["files"])
        if current["files"][path] != expected["files"][path]
    )
    report = {"ok": not (added or removed or changed), "added": added, "removed": removed, "changed": changed}
    print(json.dumps(report, indent=2))
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
