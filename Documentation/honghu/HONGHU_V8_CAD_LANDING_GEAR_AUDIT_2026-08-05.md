# 鸿鹄翼 V8 起落架 CAD 初步核查（2026-08-05）

## 项目目标与边界

以 `HHY-260113.2.stp` 和 `HHY-260113.stp` 为候选几何基准，核对 V8
起落架轮距、轴距、轮径和轮宽，并建立独立 4039 Gazebo 审核模型。
本次未修改 150 kg airframe 4028、100 kg airframe 4038、共享 mesh、
气动表、发动机表或 Gazebo 插件。

当前模型仅为 `cad_candidate_pending_field_measurement`。现场测量和机体
坐标基准注册完成之前，不得替换生产 4028/4038。

## 工具链

- Windows FreeCAD：1.1.3，`D:\Program Files\FreeCAD 1.1\bin\freecadcmd.exe`。
- CLI-Anything FreeCAD 固定提交：`39634a640cf20bc603b4faae4d31069c44821a9a`。
- Codex skill：`/mnt/c/Users/fly/.codex/skills/cli-anything-freecad/SKILL.md`。
- 隔离 harness：`/home/fly/.local/share/cli-anything-freecad/39634a640cf20bc603b4faae4d31069c44821a9a/.venv`。
- WSL 路径补丁：`Tools/honghu/cad_audit/patches/cli-anything-freecad-wsl.patch`。
- CLI 方盒/圆柱 STEP 往返尺寸最大误差：`0.0 mm`，门槛 `0.1 mm`，PASS。

CLI-Anything 只用于参数化简单几何和结构化导出。原始 STEP 装配层级、
轮子测量和子装配导出由直接 FreeCAD Python API 完成。

## STEP 与 V8 对比

两份 STEP 的六项关键尺寸完全一致，差值均为 `0.0 mm`。两主轮和前轮
的外包络圆柱拟合最大残差为 `0.184 mm`，低于 `1 mm` 门槛。

| 指标 | V8 | STEP 候选 | STEP - V8 |
|---|---:|---:|---:|
| 主轮距 | 1048.606 mm | 1151.160 mm | +102.554 mm |
| 轴距 | 1216.126 mm | 814.878 mm | -401.248 mm |
| 主轮直径 | 118.800 mm | 151.781 mm | +32.981 mm |
| 主轮宽度 | 106.000 mm | 44.322 mm | -61.678 mm |
| 前轮直径 | 87.800 mm | 140.000 mm | +52.200 mm |
| 前轮宽度 | 136.300 mm | 41.000 mm | -95.300 mm |

FreeCAD 从 `SZYA-12`、`SZYA-13` 递归选取并导出了 40 个有形状的对象，
生成 `build/honghu_cad_audit/landing_gear_selection.step`（约 15.4 MB）。

## 4039 审核模型

- Airframe：`4039_gz_honghu_wing_150kg_v8_cad_audit`。
- Gazebo 模型：`honghu_wing_150kg_v8_cad_audit`。
- 控制、质量、惯量、气动和发动机配置继承 4028；只改变起落架几何。
- 保留半透明 V8 机身作为参照；黄色圆柱表示简化支架，红/橙圆柱表示候选轮子。
- 候选绝对位置暂时保持 V8 主轮轴 x 坐标和共同地面接触平面：
  - 左主轮：`[-0.291274, 0.575580, -0.438610] m`
  - 右主轮：`[-0.291274, -0.575580, -0.438610] m`
  - 前轮：`[0.523604, 0, -0.444500] m`
- SDF 对审计 JSON 的最大几何误差：`2.8e-7 mm`，门槛 `1 mm`，PASS。

后续双重量复核发现，初版生成器移动三个轮组后仍沿用旧`base_link`残余惯性块，
会使整机重心产生亚毫米级偏移并轻微改变完整惯量。当前生成器已按平行轴定理补偿
`base_link`质心和惯量，使4039及新增4040相对各自原基线的质量一阶矩误差小于
`1.3e-8 kg·m`，原点完整惯量误差小于`5e-10 kg·m²`。因此候选模型现在严格满足
“只改变轮地几何、不改变空中质量特性”的比较条件。

该对齐方法只可靠保留 STEP 的相对尺寸。前轮绝对 x 和三根支架的机身
连接点仍需 STEP 机体基准注册或现场测量确认。

## Gazebo 验证结果

| 验证 | 结果 | 关键结果 |
|---|---|---|
| `gz sdf -k` | PASS | SDF valid |
| 120 s 静态 | PASS | Gazebo 零漂移；PX4 最大地速 0.030 m/s；最大滚转/俯仰 0.083°/0.065° |
| 低速滑跑 | PASS | 210.3 m；均速 8.24 m/s；横偏 RMS 0.153 m；保持地面接触 |
| 跑道起飞 | PASS | 43.93 m/s 离地；滑跑 535.4 m；真值俯仰峰值 7.49°；横偏 0.26 m |
| 进近与真实接地 | PARTIAL/FAIL | 接地已检测、无穿地或倾覆、最终停稳；下沉率 1.565 m/s，未通过 1 m/s 门槛 |

原报告曾把起飞滑跑`535.4 m`视为重要回归信号，后续同航线复核证明该判断使用了
不同重量/不同坐标口径。150 kg原4028在经典任务中的真实平面地面位移为
`531.1 m`，质量补偿后的4039为`535.3 m`，只增加`4.2 m（0.8%）`；4039专项
结果与同重量基线一致，不能据此认定新起落架导致滑跑距离回归。

## 未解决问题与晋级条件

1. 按 `HONGHU_V8_CAD_FIELD_MEASUREMENTS_TEMPLATE.csv` 提供实测轮距、轴距、
   三个轮子的直径/宽度，以及三个机身支架连接点。
2. 现场轮距/轴距与候选值差异不得超过 10 mm，轮径/轮宽不得超过 5 mm；
   超限时以现场值为准重新生成 4039。
3. 注册 STEP 纵向零点与 V8 CG，替换当前“固定主轮轴 x”的临时对齐策略。
4. 继续在取得实测轮胎/轮组数据后复核摩擦、滚阻和轮组质量；当前没有535 m
   新增回归证据。
5. 100/150 kg候选已完成经典完整航线、LAND、真实接地和鸭翼状态验收，但接地
   下沉率仍大于1 m/s；软着陆问题应作为独立LAND/拉平任务处理。全部现场几何门槛
   和干净机身mesh完成后，才提交4028/4038生产模型替换方案。

双重量完整复核详见`HONGHU_V8_CAD_DUAL_MASS_FLIGHT_VALIDATION_2026-08-05.md`。

## 可复现命令

```bash
python3 Tools/honghu/cad_audit/verify_baseline_isolation.py snapshot
python3 Tools/honghu/cad_audit/smoke_cli_anything_freecad.py
python3 Tools/honghu/cad_audit/run_hhy_gear_audit.py
python3 Tools/honghu/cad_audit/build_audit_model.py
python3 Tools/honghu/cad_audit/validate_audit_model.py
gz sdf -k simulation_models/models/honghu_wing_150kg_v8_cad_audit/model.sdf
python3 Tools/honghu/cad_audit/verify_baseline_isolation.py verify
```

详细 JSON、CSV、SVG、STEP、ULog 场景采样报告均位于
`build/honghu_cad_audit/`。
