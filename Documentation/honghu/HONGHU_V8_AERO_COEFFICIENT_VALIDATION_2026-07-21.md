# 鸿鹄翼 V8 六分量气动系数反算与模型一致性检验

> 日期：2026-07-21（Asia/Shanghai）
> 权威文档位置：`/home/fly/px4_reference_docs/current`
> 当前代码仓：`/home/fly/PX4-Autopilot-NewAero`
> 范围：从标准任务ULog反算六分量气动系数，并与独立V8正向模型和Gazebo插件真值比较；不做参数辨识。

## 1. 当前结论

V8气动模型的软件实现和Gazebo刚体动力学闭合已经通过完整标准任务验证。2026-07-21
最新基线加载20项 `模仿XY航线规划.plan`，从任务项0执行到18号LAND项，获得
715.348 s、14308个有效空中样本。六分量反算与独立Python正向模型的0.5 s平滑结果为：

| 系数 | 反算−独立模型bias | RMSE | 相关系数 |
|---|---:|---:|---:|
| CL | −0.0001456 | 0.0008379 | 0.999877 |
| CD | +0.00000164 | 0.00008220 | 0.999872 |
| CY | +0.0000343 | 0.0002565 | 0.999547 |
| Cl | +0.00000208 | 0.00001472 | 0.999400 |
| Cm | −0.00000341 | 0.0002319 | 0.989857 |
| Cn | −0.00000351 | 0.00001693 | 0.993070 |

因此当前证据支持：

- FRD/FLU、NED/ENU、气动力方向和气动力矩符号正确；
- 完整惯量张量、3 deg下倾推力、发动机作用点力矩和反扭矩的扣除正确；
- 插件静态表、舵效、动导数与独立Python实现一致；
- 实际Gazebo舵面反馈、发动机内部状态和诊断时间链完整；
- 先前观察到的 `CD≈+0.00855` 不是气动表错误，而是直接使用已经减去EKF偏置的
  `vehicle_acceleration` 反算造成的口径错误。

这仍然只是“仿真软件自洽性”验证，不能单独证明PDF气动数据等同于真实飞机。真实物理
准确性仍需CFD、风洞或实飞数据独立验证。

## 2. 正式基线和任务覆盖

### 2.1 日志、任务和代码

~~~text
ULog:
/home/fly/PX4-Autopilot-NewAero/build/px4_sitl_default/rootfs/log/2026-07-21/05_22_12.ulg
SHA-256:
809b3fa50e85b96f2950a849c71f9aadee6fd7627fe998c09f4517195a81c77b

任务:
/home/fly/px4_reference_docs/current/模仿XY航线规划.plan
SHA-256:
2b055389af84edbea3912576c3504c16ead5fb560f25d24fc08fc4beb5175793

固件提交:
278ce5fac16a291dda3aaa8923877b0b853ffea6
~~

关键参数为 `SYS_AUTOSTART=4028`、`SIM_GZ_SV_ZMAP=1`、
`NPFG_PERIOD=20 s`、`NAV_ACC_RAD=250 m`、`FW_R_LIM=30 deg`，空速
MIN/TRIM/MAX为32/40/55 m/s。

### 2.2 动态任务结果

~~~text
analysis_outputs/honghu_v8_standard_plan_offline_diagnostics_2ms.json
~~

- 2 ms物理步长；
- 抬轮43.914 m/s，持续离地44.163 m/s；
- 起飞段Gazebo真值最大俯仰8.227 deg；
- 跑道坐标系离地前最大横偏0.144 m；
- 任务最大当前序号18，任务失败标志为false；
- 空中高度5.005～102.173 m，空速33.349～45.264 m/s；
- 空中攻角1.887～8.962 deg，侧滑角−3.343～2.600 deg；
- 鸭翼空中保持3.989～3.992 deg，符合V3式状态逻辑；
- 运行在约5 m AGL结束，故不把触地、滑跑和鸭翼−50 deg气动刹车列为本次验收内容。

## 3. 为什么必须恢复EKF加速度偏置

### 3.1 PX4消息口径

`vehicle_acceleration.xyz` 是FRD机体系、经过标定和低通的加速度，但
`VehicleAcceleration::Run()` 在发布前还会减去 `estimator_sensor_bias.accel_bias`：

~~~text
vehicle_acceleration = filtered(calibrated_sensor_accel - EKF_accel_bias)
~~

该信号适合飞控和估计器使用，但不能直接视为未受估计器修改的物理比力。离线刚体反算
必须恢复被减去的偏置：

~~~text
f_physical_FRD = vehicle_acceleration_FRD + estimator_sensor_bias_FRD
F_aero = m * f_physical_FRD - F_prop
~~

这里加回的是EKF估计偏置，不是重力。静止、水平、FRD坐标下，加速度计Z轴仍约为
−9.8 m/s²；气动力反算不得再额外添加重力项。

### 3.2 定量证据

有效飞行区间的EKF平均加速度偏置为：

~~~text
[+0.134216, +0.023551, +0.015101] m/s²   # FRD X/Y/Z
~~

未恢复偏置时，`vehicle_acceleration` 与Gazebo位置二阶导真值的平均差为约
`[-0.1355,-0.0236,-0.0087] m/s²`，恰好导致原先的 `CD≈+0.00855` 和
`CY≈−0.00159`。恢复偏置后，与Gazebo真值比力的平均误差降为：

~~~text
[-0.000822, +0.000074, -0.000187] m/s²
~~

三轴RMSE分别为0.00458、0.00715、0.00678 m/s²。Gazebo world使用9.8 m/s²，离线
真值交叉检查也使用9.8，而不是9.80665。

结论是：此前阻力偏差来自离线测量口径，不应通过修改CD表、推力表或飞控参数补偿。

## 4. 坐标、质量惯量和动力学公式

全部反算在PDF/PX4机体系FRD完成：X前、Y右、Z下。Gazebo机体系FLU到FRD为：

~~~text
R = diag(1, -1, -1)
~~

质量和参考几何：

~~~text
m = 150 kg
S = 2.42 m²
b = 3.96 m
cbar = 0.62 m
~~

FRD完整惯量张量：

~~~text
I = [[25.86,  -0.017,  -3.520 ],
     [-0.017, 39.14,   -0.0019],
     [-3.520, -0.0019, 59.12  ]] kg·m²
~~

发动机：

~~~text
F_prop = [T cos(3°), 0, T sin(3°)]
r_prop = [-1.23, 0, -0.12] m
M_prop = r_prop × F_prop + [-Q, 0, 0]
~~

角动量方程：

~~~text
M_aero = I*omega_dot + omega × (I*omega) - M_prop
~~

风轴单位向量和系数反算：

~~~text
e_x = [cos(alpha)cos(beta), sin(beta), sin(alpha)cos(beta)]
e_y = [-cos(alpha)sin(beta), cos(beta), -sin(alpha)sin(beta)]
e_z = [-sin(alpha), 0, cos(alpha)]

CD = -(F_aero · e_x)/(qbar*S)
CY =  (F_aero · e_y)/(qbar*S)
CL = -(F_aero · e_z)/(qbar*S)
Cl = Mx/(qbar*S*b)
Cm = My/(qbar*S*cbar)
Cn = Mz/(qbar*S*b)
~~

## 5. 三条独立比较路径

### 5.1 刚体反算与独立正向模型

这是第1节主表。独立正向模型仅从ULog ground-truth重建空速、`alpha/beta`、角运动，
并读取实际Gazebo关节角；它不使用插件报告的六系数作为计算输入。

### 5.2 刚体反算与Gazebo插件真值

未经平滑的逐样本结果：

| 系数 | bias | RMSE | 相关系数 |
|---|---:|---:|---:|
| CL | −0.00000565 | 0.0003526 | 0.999974 |
| CD | +0.0000153 | 0.0001110 | 0.999786 |
| CY | +0.00000194 | 0.0000929 | 0.999947 |
| Cl | −0.000000038 | 0.0000151 | 0.999366 |
| Cm | −0.000000022 | 0.0003240 | 0.981331 |
| Cn | −0.000000101 | 0.00000638 | 0.999026 |

### 5.3 独立正向模型与Gazebo插件真值

两者六分量RMSE依次为：

~~~text
CL 0.0007587, CD 0.00006843, CY 0.0002089,
Cl 0.00001330, Cm 0.0002352, Cn 0.00001558
~~

相关系数依次为0.999903、0.999930、0.999697、0.999519、0.990762、0.994510。
小差异主要来自ULog采样重建状态与插件物理步频内部状态的带宽差异。

独立动力表与插件真值的RMSE为：推力0.00757 N、扭矩0.000655 N·m、RPM 0.0218。

## 6. 实际舵面和诊断完整性

本次日志包含36836个气动和36836个动力诊断样本，均为50 Hz。两条诊断链均满足：

- Gazebo源时间严格单调；
- 递增序列严格单调；
- 序列间隙为0；
- 当前锁步运行中到达时间减源时间的中位数、P95和最大值均为0 ms。

八个 `theta_joint` 来自Gazebo实际关节反馈。分析器独立重算四个 `delta_doc`，与插件记录
值最大差 `9.54e-7 deg`；左右升降舵最大差0.0243 deg，左右鸭翼最大差0.000370 deg。

飞行期间不启动外部 `gz topic --json-output` 观察器。动态验收只读取MAVLink，PX4和
Gazebo停止后才从ULog离线读取真值，避免再次扰动锁步实时运行。

## 7. 有效样本规则

默认筛选：

- `vehicle_land_detected.landed == false`；
- AGL不低于5 m；
- 空速不低于20 m/s；
- 动压不低于200 Pa；
- 加速度、角运动和EKF偏置均有效；
- 排除动力表输入钳位样本。

本次有效区间空速33.874～45.609 m/s、攻角1.866～8.906 deg、侧滑角
−3.312～2.568 deg，动力表钳位样本为0。0.5 s平滑只用于总体统计，不参与前向模型或
原始CSV计算。

## 8. 实现和输出文件

~~~text
Tools/honghu/analyze_honghu_v8_aero_coefficients.py
Tools/honghu/honghu_v8_aero_model.py
Tools/honghu/honghu_v8_propulsion_model.py
Tools/honghu/run_honghu_v8_dynamic_acceptance.py

msg/HonghuV8AeroState.msg
msg/HonghuV8PropulsionState.msg
src/modules/simulation/gz_bridge/GZBridge.cpp
src/modules/logger/logged_topics.cpp

analysis_outputs/honghu_v8_aero_coefficient_validation_standard_plan_truth/
  honghu_v8_aero_coefficient_summary.json
  honghu_v8_aero_coefficient_timeseries.csv
  honghu_v8_aero_coefficient_comparison.png
  honghu_v8_aero_coefficient_inputs.png
~~

## 9. 复现命令

~~~bash
cd /home/fly/PX4-Autopilot-NewAero

python3 Tools/honghu/run_honghu_v8_dynamic_acceptance.py standard \
  --step-size 0.002 \
  --timeout 1000 \
  --plan /home/fly/px4_reference_docs/current/模仿XY航线规划.plan \
  --json analysis_outputs/honghu_v8_standard_plan_offline_diagnostics_2ms.json \
  --no-assert

python3 Tools/honghu/analyze_honghu_v8_aero_coefficients.py \
  build/px4_sitl_default/rootfs/log/2026-07-21/05_22_12.ulg \
  --plan /home/fly/px4_reference_docs/current/模仿XY航线规划.plan \
  --output-dir analysis_outputs/honghu_v8_aero_coefficient_validation_standard_plan_truth
~~

正式验证不要使用以下回退项：

- `--allow-commanded-surface-fallback`：仅供没有真实关节角的历史日志；
- `--allow-estimator-biased-acceleration`：仅供没有 `estimator_sensor_bias` 的历史定性比较，
  结果会保留EKF偏置污染，不能用于判定CD/CY表是否正确。

## 10. 仍未覆盖的物理范围

1. 本验证来源仍是同一个Gazebo模型，只能证明实现闭合，不能证明PDF气动表的实机准确性。
2. 标准任务覆盖的是正常飞行包线，没有覆盖大攻角失速、`beta`超过±16 deg、极限舵偏或
   鸭翼−50 deg气动刹车。
3. LAND项进近已覆盖，但没有验收自动触地、地面减速和着陆滑跑。
4. 横侧向系数虽有多次转弯激励，但若要验证局部导数，仍建议增加安全的小幅副翼、方向舵、
   升降舵和油门分离激励。
5. 若后续更改质量惯量、气动/动力表、舵效解释、推力线、关节符号或诊断时间链，必须用
   新ULog重新生成本报告，不能沿用当前数值。
