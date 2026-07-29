# 鸿鹄翼V8 Gazebo/PX4磁力计坐标验证与修复（2026-07-29）

## 结论

QGC在航点12～13及下降/着陆阶段显示的机头左偏，主要是姿态相关的磁航向估计误差，
不是飞机真实大侧滑。官方 `gz_rc_cessna` 在完全不使用鸿鹄翼补偿时同样复现：真值航向
约90°时，EKF航向偏差均值约 `+9.79°`，磁场方向误差P95约 `4.19°`。

Gazebo Harmonic原生磁强计按历史NED方式生成磁场，却使用ENU/FLU姿态旋转；PX4官方
`[-Y,-X,+Z]`兼容映射不能在任意滚转、俯仰下构成严格三维转换。早期二维
`SIM_GZ_MAG_DCL`只能改善近水平姿态，现已删除。

## V8隔离实现

V8模型移除自身原生Harmonic磁力计，新增：

```text
src/modules/simulation/gz_plugins/honghu_v8/HonghuMagnetometerV8.cpp
src/modules/simulation/gz_plugins/honghu_v8/HonghuMagnetometerV8.hpp
```

插件使用本地PX4 WMM-2020磁场：

```text
B_NED = [0.346940371, -0.035562102, 0.325102706] gauss
B_ENU = [B_E, B_N, -B_D]
B_FLU = R_FLU_to_ENU^T * B_ENU
B_FRD = [B_FLU.x, -B_FLU.y, -B_FLU.z]
```

随后发布 `[-B_FRD.y,-B_FRD.x,B_FRD.z]`，使未修改的PX4官方回调恢复精确 `B_FRD`。
因此V3～V7和官方机型的传感器与桥接行为均不受影响。

## 验证证据

| 试验 | ULog | 关键结果 |
|---|---|---|
| 官方塞斯纳基线 | `2026-07-29/13_50_57.ulg` | EKF航向误差均值9.789°；磁场方向误差P95 4.193° |
| V8地面静态 | `2026-07-29/13_57_04.ulg` | 磁场方向误差P95 0.0264°；航向误差P95 0.054° |
| V8动态滚转 | `2026-07-29/14_03_46.ulg` | 滚转覆盖约−180°～+178°；磁场方向误差P95 0.0395° |
| V8完整任务 | `2026-07-29/14_05_18.ulg` | 到达LAND项18；磁场方向误差P95 0.195° |

完整任务中，航点12处EKF航向真值误差由修复前约 `-9.78°`降至 `-0.92°`；任务序号
14、16、18分别约为 `-0.86°`、`-0.96°`、`-1.11°`。估计虚假风速也由后段
`2.65～3.09 m/s`降至 `0.11～0.24 m/s`。剩余机头—航迹差约 `0.4～1.15°`与真实
小侧滑相容。

完整路线仍为抬轮 `44.15 m/s`、离地 `44.32 m/s`、起飞真值最大俯仰 `8.25°`、
跑道最大横偏 `0.022 m`，未因磁场修复调整气动、推进、起落架或飞控参数。

## 复现

```bash
python3 Tools/honghu/validate_gz_magnetometer_frames.py LOG.ulg \
  --field-ned-gauss 0.346940371 -0.035562102 0.325102706

python3 Tools/honghu/analyze_qgc_heading_alignment.py LOG.ulg

python3 Tools/honghu/run_honghu_v8_dynamic_acceptance.py standard \
  --plan "/home/fly/px4_reference_docs/current/模仿XY航线规划.plan" \
  --timeout 1000 --no-assert
```

详细原始统计和JSON位于：

```text
analysis_outputs/honghu_v8_magnetometer_fix/
```

本结论只验证SITL磁场坐标链，不包含实机磁罗盘安装、硬铁/软铁和电机磁干扰模型。
