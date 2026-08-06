# Honghu V8 landing-gear CAD audit

This workflow treats both HHY STEP files as CAD candidates and leaves the 4028
and 4038 baselines untouched. JSON-in-YAML configuration avoids adding a PyYAML
runtime dependency.

## Run

```bash
python3 Tools/honghu/cad_audit/verify_baseline_isolation.py snapshot
python3 Tools/honghu/cad_audit/smoke_cli_anything_freecad.py
python3 Tools/honghu/cad_audit/run_hhy_gear_audit.py
python3 Tools/honghu/cad_audit/build_audit_model.py
python3 Tools/honghu/cad_audit/validate_audit_model.py
python3 Tools/honghu/cad_audit/verify_baseline_isolation.py verify
```

Generated artifacts are placed in `build/honghu_cad_audit/`:

- `cad_measurements.json`: raw STEP objects, normalized dimensions and candidate SDF values.
- `v8_vs_step.csv`: dimension-by-dimension V8 comparison.
- `gear_comparison.svg`: side/front/top schematic overlay.
- `landing_gear_selection.step`: FreeCAD export of `SZYA-12` and `SZYA-13` descendants.
- `cli_smoke/toolchain_report.json`: pinned harness and 0.1 mm smoke-test evidence.

The candidate alignment deliberately preserves the V8 main axle x coordinate
and ground-contact plane. Do not promote absolute attachment locations until a
STEP-to-aircraft datum registration or field measurements are available.

Moving a wheel group would normally shift the assembled CG and inertia. The
builder compensates the residual `base_link` inertial block with the parallel-axis
theorem so a CAD candidate changes wheel/contact geometry without changing the
baseline whole-aircraft mass properties.

The builder also supports an external baseline for the isolated 100 kg 4040
candidate:

```bash
python3 Tools/honghu/cad_audit/build_audit_model.py \
  --baseline-sdf /home/fly/PX4-Autopilot-NewAero-100kg/simulation_models/models/honghu_wing_100kg_v8_xiangyi_test/model.sdf \
  --output /home/fly/PX4-Autopilot-NewAero-100kg/simulation_models/models/honghu_wing_100kg_v8_cad_audit/model.sdf \
  --model-name honghu_wing_100kg_v8_cad_audit \
  --asset-model honghu_wing_150kg_v8
```
