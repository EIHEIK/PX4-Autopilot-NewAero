# 鸿鹄翼 NewAero CAD核查迁移、试验机与日志归档记录

日期：2026-08-06

关联Codex任务：`019fcfcb-6857-77e3-88ed-22e34c18c31e`

公共Git起点：`2a644a5f1e37ab0357646be26e05ef1df80e744c`

## 1. 本次整理结论

1. 150 kg CAD核查机型4039已从临时`PX4-Autopilot-NewAero-CAD-audit`工作现场迁入`PX4-Autopilot-NewAero-150kg`，作为150 kg分支试验机长期保留。
2. 100 kg CAD核查机型4040继续保留在`PX4-Autopilot-NewAero-100kg`，作为100 kg分支试验机。
3. 生产/稳定入口仍是150 kg的4028和100 kg的4038；本次没有覆盖它们，也没有修改共享气动表、发动机表或mesh。
4. `CAD-audit` worktree不再作为日常机型入口。它暂时作为原始核查过程和未提交历史现场保留；只有在4039、审计工具、报告和证据都进入150 kg分支并完成提交/回归后，才可移除该worktree。
5. 4028/4039、4038/4040四条正式对照ULog已经按重量归档到worktree之外的`PX4-Autopilot-NewAero-flight-data`。

## 2. 当前型号与worktree边界

| 目录/分支 | 稳定入口 | 核查入口 | 职责 |
|---|---:|---:|---|
| `PX4-Autopilot-NewAero/main` | 无单独型号承诺 | 无 | 公共集成、共享接口和文档镜像 |
| `PX4-Autopilot-NewAero-100kg` / `variant/honghu-v8-100kg` | 4038 | 4040 | 100 kg控制、模型和CAD候选试验 |
| `PX4-Autopilot-NewAero-150kg` / `variant/honghu-v8-150kg` | 4028 | 4039 | 150 kg生产基线、回归和CAD候选试验 |
| `PX4-Autopilot-NewAero-CAD-audit` / `codex/honghu-v8-cad-audit` | 不再使用 | 历史4039来源 | 临时迁移源，待提交与核对完成后退役 |

型号规则：

- 4028/4038用于稳定飞行和正式生产基线回归；
- 4039/4040允许独立修改轮地几何、起落架候选和CAD注册，不得反向覆盖稳定入口；
- CAD候选只有在现场尺寸、绝对坐标注册、轮胎参数和干净机身mesh均通过后，才可另立方案升级生产模型。

## 3. 任务019fcfcb中的模型核查内容

### 3.1 CAD资料与工具链

`px4_reference_docs/current/HHYmodel`包含CATIA装配资料和两份整机STEP：

- `HHY-260113.stp`：完整对象更多，用于复核装配完整性；
- `HHY-260113.2.stp`：文件较轻，用于主要自动测量；
- `SZYA-12`、`SZYA-13`：起落架相关子装配，可递归选择并独立导出。

不需要为本轮尺寸核查安装SolidWorks或CATIA。当前链路为：

```text
Codex/WSL
  -> Windows FreeCADCmd 1.1.3
  -> 直接FreeCAD Python API读取STEP
  -> WSL侧完成坐标归一化、V8对比、JSON/CSV/SVG输出
```

CLI-Anything FreeCAD固定到提交`39634a640cf20bc603b4faae4d31069c44821a9a`，只用于简单参数化几何和结构化导出。WSL调用Windows FreeCAD所需路径补丁已保存在：

```text
PX4-Autopilot-NewAero-150kg/Tools/honghu/cad_audit/patches/
cli-anything-freecad-wsl.patch
```

方盒/圆柱STEP往返最大尺寸误差为`0.0 mm`，通过`0.1 mm`工具链门槛。

### 3.2 STEP与原V8轮地几何差异

两份整机STEP的六项关键尺寸差值均为`0.0 mm`。轮子外包络圆柱拟合最大残差为`0.184 mm`，通过`1 mm`门槛。

| 指标 | 原V8 | STEP候选 | STEP - V8 |
|---|---:|---:|---:|
| 主轮距 | 1048.606 mm | 1151.160 mm | +102.554 mm |
| 轴距 | 1216.126 mm | 814.878 mm | -401.248 mm |
| 主轮直径 | 118.800 mm | 151.781 mm | +32.981 mm |
| 主轮宽度 | 106.000 mm | 44.322 mm | -61.678 mm |
| 前轮直径 | 87.800 mm | 140.000 mm | +52.200 mm |
| 前轮宽度 | 136.300 mm | 41.000 mm | -95.300 mm |

候选轮心（Gazebo FLU、相对整机质心）：

```text
left main   [-0.291274, +0.575580, -0.438610] m
right main  [-0.291274, -0.575580, -0.438610] m
nose        [+0.523604,  0,        -0.444500] m
```

候选保持主轮轴`x=-0.291274 m`和共同地面接触面`z=-0.5145 m`。因此它只可靠保留STEP相对几何；前轮绝对纵向位置仍取决于尚未完成的STEP机体基准注册。

### 3.3 质量惯量补偿

初版候选只移动轮组，未补偿整机一阶质量矩和完整惯量。后续生成器已按平行轴定理反算`base_link`残余质量特性，使4039/4040相对各自原基线达到：

```text
最大一阶质量矩误差    < 1.3e-8 kg*m
最大原点完整惯量误差  < 5.0e-10 kg*m^2
```

因此候选对比只改变轮地几何，不暗中改变100 kg或150 kg空中质量特性。

### 3.4 双重量飞行结果

同一`模仿XY航线规划.plan`、2 ms物理步长下：

| 构型 | 原模型离地TAS | CAD候选离地TAS | 高度误差RMS 原/候选 | 接地下沉率 原/候选 | 结论 |
|---|---:|---:|---:|---:|---|
| 150 kg，4028/4039 | 43.938 | 44.019 m/s | 0.923/0.922 m | 2.078/1.922 m/s | 空中和起飞无退化 |
| 100 kg，4038/4040 | 40.362 | 40.441 m/s | 0.531/0.525 m | 2.894/2.890 m/s | 空中和起飞无退化 |

两种候选都完成全部任务、LAND、真实接地，并记录鸭翼空中约`+6°`、接地后回中、约5 s后进入`-50°`气动刹车。没有穿地或倾覆。

原报告曾把4039约535 m滑跑视作明显回归。与同重量、同任务4028复核后，4028约531.1 m、4039约535.3 m，仅相差约0.8%；此前判断混用了重量或位移口径，现已更正。

当前不因CAD候选调整飞控参数。两个重量仍有接地下沉率大于`1 m/s`的问题，应作为独立LAND拉平/软着陆任务处理。

## 4. 迁入150 kg分支的文件

```text
PX4-Autopilot-NewAero-150kg/
├── ROMFS/px4fmu_common/init.d-posix/airframes/
│   └── 4039_gz_honghu_wing_150kg_v8_cad_audit
├── simulation_models/models/
│   └── honghu_wing_150kg_v8_cad_audit/
├── Tools/honghu/cad_audit/
├── Documentation/honghu/HONGHU_V8_CAD_*.md
└── analysis_outputs/honghu_v8_cad_audit_2026-08-05/
    └── runtime_acceptance/（原CAD工作现场的专项运行结果）
```

150 kg的airframe CMake清单已登记4039。迁移采用复制后校验，不从临时worktree删除原始文件。

100 kg分支继续保留：

```text
PX4-Autopilot-NewAero-100kg/
├── ROMFS/.../4040_gz_honghu_wing_100kg_v8_cad_audit
└── simulation_models/models/honghu_wing_100kg_v8_cad_audit/
```

## 5. 正式日志归档

```text
/home/fly/PX4-Autopilot-NewAero-flight-data/
├── 100kg/2026-08-05-cad-audit-validation/
│   ├── ulog/4038_2026-08-05_03_32_08.ulg
│   ├── ulog/4040_2026-08-05_09_41_18.ulg
│   ├── parameters/
│   ├── airframes/
│   ├── mission/
│   ├── SUMMARY.md
│   └── SHA256SUMS
└── 150kg/2026-08-05-cad-audit-validation/
    ├── ulog/4028_2026-08-05_09_14_25.ulg
    ├── ulog/4039_2026-08-05_09_28_01.ulg
    ├── parameters/
    ├── airframes/
    ├── mission/
    ├── SUMMARY.md
    └── SHA256SUMS
```

参数文件由`ulog_params -i -f qgc`直接从对应ULog导出，不依赖worktree当前运行时参数文件。归档ULog的SHA-256与双重量报告一致。

## 6. CAD-audit worktree生命周期

`PX4-Autopilot-NewAero-CAD-audit`当前为“待退役迁移源”，不是无用目录，也不是独立仓库。移除前必须同时满足：

1. 150 kg分支中的4039、模型、工具和报告已提交；
2. 4039完成至少SDF检查、构建和一条完整任务回归；
3. 参考文档和日志归档中的哈希检查通过；
4. `git status`确认临时worktree不再有唯一且未迁移的内容；
5. 用户明确批准删除worktree。

满足前述条件后可用Git worktree正常移除，禁止直接删除目录。

## 7. 尚未完成的模型核查

- 现场测量轮距、轴距、轮径、轮宽和支架连接点；
- STEP机体绝对纵向零点与V8质心注册；
- 真实轮胎摩擦、滚阻、轮组质量和转动惯量；
- 从STEP重新导出不含旧起落架支架的干净机身mesh；
- 侧风、摩擦变化和CG偏移回归；
- LAND拉平和接地下沉率优化。

在这些工作完成前，4039/4040的状态始终是`cad_candidate_pending_field_measurement`。
