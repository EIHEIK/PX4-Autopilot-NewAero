#!/usr/bin/env python3
"""Generate an isolated 100 kg Xiangyi-condition model from the stable V8.

The source model is read-only. Exact replacements are deliberately checked so
an upstream model change cannot silently produce a malformed test derivative.
"""

from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SOURCE_DIR = ROOT / "simulation_models/models/honghu_wing_150kg_v8"
OUTPUT_DIR = ROOT / "simulation_models/models/honghu_wing_100kg_v8_xiangyi_test"
SOURCE_SDF = SOURCE_DIR / "model.sdf"
OUTPUT_SDF = OUTPUT_DIR / "model.sdf"
GENERATOR = ROOT / "Tools/honghu/generate_honghu_v8_model.py"

def replace_once(text: str, old: str, new: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"expected exactly one occurrence of {old!r}, found {count}")
    return text.replace(old, new, 1)


def load_generator():
    spec = importlib.util.spec_from_file_location("honghu_v8_generator_for_xiangyi", GENERATOR)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load V8 generator: {GENERATOR}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


source_bytes = SOURCE_SDF.read_bytes()
source_sha256_before = hashlib.sha256(source_bytes).hexdigest()
text = source_bytes.decode("utf-8")

text = replace_once(
    text,
    '<model name="honghu_wing_150kg_v8">',
    '<model name="honghu_wing_100kg_v8_xiangyi_test">',
)

# Keep the complete 73 kg airframe identical to production.  Only the separate
# ballast link changes mass, position and inertia.  The generator uses the same
# physical ballast package at constant inertia per unit mass and solves its
# location so the complete 100 kg model closes at x=-1.57 m.
generator = load_generator()
text = replace_once(
    text,
    generator.ballast_xml(generator.TARGET_150_MASS),
    generator.ballast_xml(100.0),
)
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
    "  <!-- ISOLATED XIANGYI TEST DERIVATIVE: 73 kg full-fuel aircraft plus\n"
    "       27 kg adjustable ballast; +X FRD reaction-torque A/B case.\n"
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
ballast_mass, ballast_com, _ = generator.ballast_properties(100.0)
_, _, target_inertia = generator.target_mass_properties(100.0)
print(f"base_aircraft_mass_kg={generator.BASE_73_MASS:.12g}")
print("base_aircraft_cg_pdf_frd_m=" + ",".join(f"{value:.12g}" for value in generator.BASE_73_CG_PDF_FRD))
print("target_assembled_cg_pdf_frd_m=" + ",".join(f"{value:.12g}" for value in generator.TARGET_ASSEMBLED_CG_PDF_FRD))
print(f"ballast_mass_kg={ballast_mass:.12g}")
print("ballast_com_gz_m=" + ",".join(f"{value:.12g}" for value in ballast_com))
print("target_inertia_gz_kgm2=" + ",".join(
    f"{target_inertia[i][j]:.12g}" for i, j in ((0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2))
))
print("production_source_unchanged=true")
