# Honghu Wing 100 kg V8 CAD audit model

This isolated model combines the current 100 kg compositional mass model and
4038 control configuration with the STEP-derived landing-gear geometry from the
2026-08-05 CAD audit. It does not replace 4038.

The wheel geometry is still marked `cad_candidate_pending_field_measurement`.
Its main axle x coordinate and ground-contact plane remain temporary alignment
anchors; field wheel dimensions and the STEP-to-aircraft longitudinal datum
must be confirmed before production use.

Launch with:

```bash
make px4_sitl gz_honghu_wing_100kg_v8_cad_audit
```
