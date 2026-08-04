# 鸿鹄翼V8组合式质量、惯量与配重模型

日期：2026-08-03
适用代码库：`/home/fly/PX4-Autopilot-NewAero`

## 1. 结论

V8质量模型已由“每个重量版本直接替换整机惯性参数”改为：

```text
73 kg机内满油基础飞机
  + 显式舵面和滚轮子链接
  + 可调固定配重链接 adjustable_ballast
  = 100 kg或150 kg完整模型
```

100 kg和150 kg的目标重心均为PDF/PX4 FRD鼻端原点下
`(-1.57, 0, 0) m`。气动力矩参考点仍为同一点，气动表、发动机表、
起落架、控制参数和鸭翼状态机均未因本次改动而调整。

## 2. 数据口径与工程假设

Word表3给出的73 kg机内满油状态为：

```text
m = 73 kg
Ixx/Iyy/Izz = 25.33/30.81/50.98 kg m^2
Ixy/Ixz/Iyz = -0.021/-2.592/-0.0002 kg m^2（Word/FRD记录口径）
CGz = -0.03 m
```

Word中的`CGx=-1.56 m`已按项目决定视为笔误，不再使用。根据“73 kg基础机
重心比-1.57 m更靠近机尾”的要求，在缺少实测值时采用最小明确偏置：

```text
73 kg基础机CG = (-1.58, 0, -0.03) m，PDF FRD
```

即纵向比100/150 kg目标重心向后10 mm。该值集中定义在
`generate_honghu_v8_model.py`的`BASE_73_CG_PDF_FRD`，后续取得实测值时只
修改这一处并重新生成模型。

## 3. 配重求解

设73 kg基础机在Gazebo参考坐标中的重心为`r73`，目标总重心为`rt`，目标质量
为`M`，配重质量为`mb=M-73`，则：

```text
rb = (M*rt - 73*r73) / mb
```

当前`rt=(0,0,0)`，得到：

| 版本 | 配重质量 | 配重中心，Gazebo FLU，相对x=-1.57 m参考点 |
|---|---:|---:|
| 100 kg | 27 kg | `(0.027037037, 0, -0.081111111) m` |
| 150 kg | 77 kg | `(0.009480519, 0, -0.028441558) m` |

配重是无碰撞的内部固定链接，具有独立质量、位置、完整惯量张量和可见橙色
调试外形。Gazebo外力仍在`base_link`参考点施加；`AddWorldWrench`的定义是将
力和力矩作用在链接原点，因此将来目标CG相对气动参考点移动时，力臂效应由刚体
动力学自然产生，不需要在气动插件中重复补偿。

## 4. 惯量构造

73 kg基础飞机的总惯量严格保留Word表3。生成器先扣除已显式建模的舵面和车轮，
反算残余`base_link`的质量、重心和惯量，避免重复计数。

77 kg完整配重的内禀惯量由150 kg Word总惯量反算，使150 kg装配结果严格为：

```text
I_GZ_150 =
[25.86   0.017   3.5200]
[ 0.017 39.14   -0.0019]
[ 3.520 -0.0019 59.12  ] kg m^2
```

100 kg使用相同配重包络并保持单位质量惯量不变，27 kg配重内禀惯量按`27/77`
缩放。平行轴装配后的100 kg惯量为：

```text
I_GZ_100 =
[25.714298926   0.019597403   2.983554188]
[ 0.019597403  33.951414391  -0.000796104]
[ 2.983554188  -0.000796104  53.856336244] kg m^2
```

旧的73/150 kg整机惯量线性插值没有继续使用。把该插值目标与Word给出的73 kg
垂向重心同时强制施加到单一27 kg配重上，会反算出负的主惯量，物理上不可实现。
组合式结果因此优先于旧插值结果。

## 5. 修改和复现入口

主要文件：

```text
Tools/honghu/generate_honghu_v8_model.py
Tools/honghu/prepare_xiangyi_v8_test_model.py
Tools/honghu/check_honghu_v8.py
simulation_models/models/honghu_wing_150kg_v8/model.sdf
simulation_models/models/honghu_wing_100kg_v8_xiangyi_test/model.sdf
```

重建和静态验收：

```bash
cd /home/fly/PX4-Autopilot-NewAero
python3 Tools/honghu/generate_honghu_v8_model.py
python3 Tools/honghu/prepare_xiangyi_v8_test_model.py
python3 Tools/honghu/check_honghu_v8.py
gz sdf -k simulation_models/models/honghu_wing_150kg_v8/model.sdf
gz sdf -k simulation_models/models/honghu_wing_100kg_v8_xiangyi_test/model.sdf
```

需要改变目标重心时，修改`TARGET_ASSEMBLED_CG_PDF_FRD`；需要修正73 kg基础机
实测重心时，修改`BASE_73_CG_PDF_FRD`。不得直接修改生成后的SDF惯性块。

## 6. 验证结果

### 6.1 静态与接口检查

- 两个SDF均通过`gz sdf -k`；
- 73 kg基础机、配重、舵面和车轮的质量矩及完整惯量闭合；
- 100/150 kg装配重心均回到`(-1.57,0,0) m`；
- 150 kg Word目标惯量严格闭合；
- 100 kg组合惯量严格闭合；
- 917项气动表、符号、连续性和配平契约全部通过。

150 kg和100 kg静置约62 s时，Gazebo真值水平漂移和高度变化均为0，姿态远小于
0.001°，说明固定配重没有引入重力矩、接触偏置或关节松动。

### 6.2 150 kg飞行回归

150 kg总质量、CG和总惯量与改动前相同。起飞—爬升—稳定飞行自动验收PASS：

| 指标 | 结果 |
|---|---:|
| 真值离地空速 | 43.99 m/s（地速44.11 m/s） |
| 起飞真值俯仰峰值 | 7.46° |
| 离地前横向偏差 | 0.039 m |
| 稳定飞行空速 | 39.71～40.49 m/s |
| 稳定飞行滚转峰值 | 29.34° |
| 鸭翼实际角 | 6.003°附近 |

### 6.3 100 kg经典航线

100 kg完成经典任务至LAND项18并真实接地：

| 指标 | 结果 |
|---|---:|
| 真值离地空速 | 40.34 m/s（地速40.50 m/s） |
| 起飞真值俯仰峰值 | 6.16° |
| 离地前横向偏差 | 0.0083 m |
| 空中真值滚转峰值 | 26.27° |
| 空中真值俯仰峰值 | 8.66° |
| 空速范围 | 30.79～46.63 m/s |
| 接地后最大滚转 | 3.46° |
| 地面穿透 | 无，最低约-0.00014 m数值容差 |

自动报告唯一FAIL项为接地下沉率`3.390 m/s`。改动前100 kg基线为
`3.396 m/s`，其他旧候选也稳定在约`3.36～3.40 m/s`，因此它不是配重模型回归，
而是现有经典航线LAND/拉平参数的独立遗留问题。

新旧100 kg日志的统一10 s高通统计显示：转弯段俯仰波动标准差约由1.32°降到
1.10°，高度误差RMS约由1.20 m降到1.01 m；全巡航俯仰波动约降低6%。直线段
高度误差约增加4%，属于单架次小差异。没有证据表明新组合惯量加剧空中振荡。

验证报告：

```text
analysis_outputs/honghu_v8_mass_ballast_2026-08-03/150kg_static.json
analysis_outputs/honghu_v8_mass_ballast_2026-08-03/100kg_static.json
analysis_outputs/honghu_v8_mass_ballast_2026-08-03/150kg_flight.json
analysis_outputs/honghu_v8_mass_ballast_2026-08-03/100kg_standard_touchdown.json
```

对应100 kg完整ULog：

```text
build/px4_sitl_default/rootfs/log/2026-08-03/07_37_42.ulg
```

## 7. 当前边界

- 73 kg纵向重心`-1.58 m`是等待实测确认的工程假设；
- 100 kg惯量是组合式推导值，不是直接测量值；
- 当前配重按同一包络、单位质量惯量不变处理；
- 本次没有调整负迎角舵效表、气动表、发动机表或飞控参数；
- 约3.4 m/s接地下沉率应通过独立LAND/拉平试验解决，不能用修改质量惯量掩盖。
