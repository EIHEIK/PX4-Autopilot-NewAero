# Honghu V8 CAD landing-gear audit model

This model is an isolated geometry candidate for airframe 4039. It reuses the
validated 150 kg V8 meshes, aerodynamic tables and propulsion tables by URI;
those shared assets are not copied or modified.

The body is intentionally semi-transparent because the old three struts are
baked into `body.dae`. Red/orange cylinders show STEP-derived wheel candidates,
and yellow cylinders show simplified replacement struts. The main axle x datum
and common ground plane are held at the V8 values until aircraft datum and field
measurements are supplied.

Regenerate after a CAD audit:

```bash
python3 Tools/honghu/cad_audit/build_audit_model.py
```

Validate:

```bash
gz sdf -k simulation_models/models/honghu_wing_150kg_v8_cad_audit/model.sdf
make px4_sitl gz_honghu_wing_150kg_v8_cad_audit
```
