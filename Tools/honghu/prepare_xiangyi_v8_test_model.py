#!/usr/bin/env python3
"""Generate an isolated 100 kg Xiangyi-condition model from the stable V8.

The source model is read-only. Exact replacements are deliberately checked so
an upstream model change cannot silently produce a malformed test derivative.
"""

from __future__ import annotations

import hashlib
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SOURCE_DIR = ROOT / "simulation_models/models/honghu_wing_150kg_v8"
OUTPUT_DIR = ROOT / "simulation_models/models/honghu_wing_100kg_v8_xiangyi_test"
SOURCE_SDF = SOURCE_DIR / "model.sdf"
OUTPUT_SDF = OUTPUT_DIR / "model.sdf"


def replace_once(text: str, old: str, new: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"expected exactly one occurrence of {old!r}, found {count}")
    return text.replace(old, new, 1)


source_bytes = SOURCE_SDF.read_bytes()
source_sha256_before = hashlib.sha256(source_bytes).hexdigest()
text = source_bytes.decode("utf-8")

text = replace_once(
    text,
    '<model name="honghu_wing_150kg_v8">',
    '<model name="honghu_wing_100kg_v8_xiangyi_test">',
)
text = replace_once(text, "<mass>149.16</mass>", "<mass>99.16</mass>")
text = replace_once(
    text,
    "<!-- Front-view CCW propeller: omega is +X, airframe reaction torque is -X. -->\n"
    "      <propeller_rotation_sign>1</propeller_rotation_sign>",
    "<!-- Xiangyi-log A/B hypothesis only: airframe reaction torque is +X FRD. -->\n"
    "      <propeller_rotation_sign>-1</propeller_rotation_sign>",
)

# The test derivative contains no copied CAD or tables. Explicit model URIs
# keep those immutable assets sourced from the production V8 directory.
text = text.replace(
    "<uri>meshes/",
    "<uri>model://honghu_wing_150kg_v8/meshes/",
)

marker = "  <!-- V8 coordinate contract:"
test_notice = (
    "  <!-- ISOLATED XIANGYI TEST DERIVATIVE: total mass 100 kg; V8/PDF inertia\n"
    "       retained as an explicit assumption; +X FRD reaction-torque A/B case.\n"
    "       Generated from the stable 150 kg model; never edit it as production. -->\n"
)
if marker not in text:
    raise RuntimeError("source coordinate-contract marker not found")
text = text.replace(marker, test_notice + marker, 1)

OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
OUTPUT_SDF.write_text(text, encoding="utf-8")
(OUTPUT_DIR / "model.config").write_text(
    """<?xml version="1.0"?>
<model>
  <name>Honghu Wing V8 Xiangyi 100 kg Isolated Test</name>
  <version>8.0-xiangyi-test</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>Honghu PX4 simulation team</name></author>
  <description>
    Test-only 100 kg derivative for reproducing Xiangyi flight conditions.
    It reuses the stable V8 assets and is not a production aircraft model.
  </description>
</model>
""",
    encoding="utf-8",
)

source_sha256_after = hashlib.sha256(SOURCE_SDF.read_bytes()).hexdigest()
if source_sha256_after != source_sha256_before:
    raise RuntimeError("production V8 model changed while generating the test derivative")

print(f"source={SOURCE_SDF}")
print(f"source_sha256={source_sha256_before}")
print(f"output={OUTPUT_SDF}")
print(f"output_sha256={hashlib.sha256(OUTPUT_SDF.read_bytes()).hexdigest()}")
print("test_total_mass_kg=100.0")
print("production_source_unchanged=true")
