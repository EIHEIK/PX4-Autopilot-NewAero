# Honghu V8 Xiangyi 100 kg isolated test model

This directory is a test-only derivative of `honghu_wing_150kg_v8`.

- total mass: 100 kg (`base_link` 99.16 kg plus 0.84 kg child links);
- inertia: the current PDF/V8 tensor is retained because Xiangyi's 100 kg
  inertia was not provided;
- aerodynamic and propulsion tables: read from the production V8 model;
- propeller reaction torque: isolated `+X FRD` hypothesis;
- production model and airframe: not modified by the generator.

Regenerate it with:

```bash
python3 Tools/honghu/prepare_xiangyi_v8_test_model.py
```

Run the isolated airframe directly:

```bash
make px4_sitl gz_honghu_wing_100kg_v8_xiangyi_test
```

The stable production entry remains:

```bash
make px4_sitl gz_honghu_wing_150kg_v8
```
