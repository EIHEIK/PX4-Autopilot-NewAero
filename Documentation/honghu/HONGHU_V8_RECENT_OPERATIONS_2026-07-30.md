# 鸿鹄翼 V8 近期操作与当前状态汇总（2026-07-30）

## 1. 文档目的

本文汇总自2026-07-29参考文档整理后，到2026-07-30用户最新复飞确认之间的主要对话结论、代码操作、仿真证据和当前边界。

事实源优先级仍为：

1. `/home/fly/px4_reference_docs/current/HONGHU_V8_PROJECT_REFERENCE_2026-07-15.md`
2. 本文及同目录专项报告
3. NewAero仓库内`Documentation/honghu/`同步副本
4. 原始ULog、JSON、MATLAB结果和历史聊天记录

本文不替代各专项报告，也不复制原始翔仪CSV、ULog或大体积分析输出。

## 2. 当前工作仓和版本关系

当前唯一生产开发仓：

```text
/home/fly/PX4-Autopilot-NewAero
branch: main
GitHub private: git@github.com:EIHEIK/PX4-Autopilot-NewAero.git
```

`/home/fly/PX4-Autopilot-canard-2026.6.2`已停留在NewAero迁移前后的较早阶段，不再作为最近修改的事实源。V3代码和V3机型没有在本轮修改。

本轮GitHub同步前的本地HEAD为`278ce5fac1`，远端`origin/main`为`51e344baa1`；本地已有“首次迁移说明”和“前轮执行器修复”两个未推送提交，另有本轮完整未提交工作树。

## 3. 翔仪模型核对和100 kg隔离验证

近期完成了翔仪外部仿真的离线气动与闭环对比：

- 确认翔仪仿真质量为100 kg，而生产V8仍为150 kg；
- 用翔仪RFU到PX4/PDF FRD的运动学自检后反算六分量；
- 将翔仪约65%油门按1.25比例映射到V8发动机表约81.25%输入；
- MATLAB R2025a脚本保持单文件、顺序执行、不依赖自编函数；
- 参数Word表3提供73/150 kg质量特性，100 kg隔离模型和离线反算已改为线性插值惯量；
- 插值后`CL/CD/Cl`在当前共同假设下仍条件一致，`CY/Cn`激励不足，`Cm`仍对插值惯量和推进力矩敏感；
- 100 kg较低离地速度主要可由质量平方根缩放解释，不要求假设升力表不同；
- 新增4038和`honghu_wing_100kg_v8_xiangyi_test`仅用于隔离复现，不修改150 kg生产模型。

对应报告：

```text
HONGHU_V8_XIANGYI_AERO_COMPARISON_2026-07-28.md
HONGHU_V8_XIANGYI_AERO_COMPARISON_MATLAB_2026-07-29.md
HONGHU_V8_XIANGYI_CLOSED_LOOP_COMPARISON_2026-07-29.md
HONGHU_V8_LONGITUDINAL_STABILITY_OPTIMIZATION_2026-07-29.md
```

## 4. 六分量真值诊断和实时性边界

V8增加Gazebo实际关节角、气动状态和推进状态到uORB/ULog的离线诊断链。导致消息积压的在线`gz topic`观察器已经移除，飞行期间只保留飞控所需实时链路；系数闭合在仿真停止后离线完成。

完整20项任务的独立正向模型与动力学反算高度闭合，证明当前插件、坐标、舵偏和动力学软件链自洽。该结论不等于真实飞机气动参数已经由试飞验证。

## 5. 坐标、磁场和QGC航向

使用官方`gz_rc_cessna`完成对照，确认官方Harmonic历史磁场路径在当前Gazebo ENU/FLU姿态下也会出现约10°级航向偏差，不是V8方向舵或气动独有问题。

V8采用隔离的`HonghuMagnetometerV8`：

- WMM NED磁场完整旋转到机体FRD；
- 对未修改的PX4官方`[-Y,-X,+Z]`桥接做逆表示；
- 不修改PX4官方通用磁力计桥接；
- 不影响V3～V7和官方机型。

标准任务中磁场方向误差P95降到约0.195°，QGC箭头方向恢复正常。详见`HONGHU_V8_MAGNETOMETER_FRAME_VALIDATION_2026-07-29.md`。

## 6. 鸭翼、起飞姿态和纵向控制

V8鸭翼保持V3派生的独立状态机：

- 起飞进入CLIMBOUT后展开；
- 起飞、巡航和降落进近保持约+6°；
- 不参与实时俯仰分配；
- 接地判定和延时满足后才进入-50°空气刹车。

后置螺旋桨几何擦地角约10°，起飞真值俯仰验收收紧为8.5°。当前隔离起飞典型结果为离地约42.7 m/s、真值俯仰峰值约8.0°。

150 kg纵向分析确认：直线内环基本稳定，主要波动集中在转弯和航段交接。最终保留积分器并采用`FW_T_PTCH_DAMP=0.30`，未使用会使部分大转弯退化的`FW_T_SPDWEIGHT=0.7`组合。

## 7. 地面碰撞和降落

“接地后穿到地面以下”不是起落架几何或磁场导致，而是旧无限平面在DART/FCL宽相位中的有效碰撞范围没有覆盖视觉显示的30 km范围。

V8 world现使用`30000×30000×1 m`有限实体地面盒，顶面仍为`z=0`。修复后：

- 原失败位置静置稳定；
- 2 ms滑跑通过；
- 接地后不再穿透，最低高度仅毫米以下的数值量级；
- 刚性支柱和实体滚轮保留，没有重新引入复杂悬架。

当前自动降落仍未完成最终验收：接地下沉率约1.5 m/s，高于小于1 m/s目标；无轮刹车时测试窗口内不能完全停稳。

## 8. 起飞、巡航、降落纵向阶段切换

现有实现不是三套完整PID，而是共用控制器，只对正俯仰角权限、正俯仰率权限和起飞/降落特殊前馈进行阶段管理。P/I/D、TECS主体参数和积分器仍然共用并保持启用。

当前参数：

```text
巡航: FW_P_LIM_MAX=10 deg, FW_P_RMAX_POS=10 deg/s
起飞: RWTO_PMAX=8 deg, FW_P_RMAX_TKO=6 deg/s
降落: FW_LND_PMAX=8 deg, FW_P_RMAX_LND=6 deg/s
触发: FW_P_TKO_HGT=50 m, FW_P_LND_HGT=50 m
时间: FW_P_TRANS_DUR=5 s
安全: FW_P_RMAX_SLEW=2 deg/s^2
前馈: FW_PR_FF=0.90, FW_PR_FF_RWTO=6.6, FW_PR_FF_LND=6.6
```

高度不再连续映射参数，只负责一次性触发：

- 自动起飞记录起飞点高度；首次超过50 m后，在5 s smoothstep内从起飞权限释放到巡航权限；
- 起飞状态跨TAKEOFF任务项保持，避免任务先切换导致权限瞬间放开；
- LAND任务首次低于50 m后，在5 s内从巡航权限收紧到降落权限；
- 触发后不再随高度回升或抖动反向变化；
- 降落阶段优先于从未超过50 m的低空起飞状态。

曾发现动态任务在地面等待时短暂预发布LAND项会取消待执行起飞状态，导致45.3 m/s才抬轮、俯仰峰值10.36°。现已禁止“仍在地面且起飞未完成”的LAND预发布取消起飞状态。修正后起飞恢复正常。

## 9. 本轮验证结果

| 验证 | 当前结果 |
|---|---|
| PX4 SITL全量编译 | PASS |
| V8静态契约 | PASS |
| 气动表/符号/连续性/配平 | 917项PASS |
| 50 m起飞触发 | 50.23 m触发，约4.94 s后完成 |
| 起飞时间渐变 | 权重0.9998→0.0001，单调 |
| 隔离起飞 | 42.71 m/s离地，俯仰峰值8.01° |
| 标准航线前150 s | 到任务项3，姿态和空速有界；主动超时，不代表完整任务PASS |
| 降落时间渐变 | 低于50 m后约4.94 s完成，短时高度回升不影响权重单调性 |
| 接地 | 约1.50 m/s，不穿地；软着陆指标未通过 |
| 用户最新QGC复飞 | 用户肉眼确认本次没有明显问题；属于操作确认，不替代量化验收 |

关键JSON位于本地：

```text
analysis_outputs/honghu_v8_phase_pitch_tuning/takeoff_time_trigger_50m_5s_candidate2.json
analysis_outputs/honghu_v8_phase_pitch_tuning/standard_takeoff_time_transition_50m_5s.json
analysis_outputs/honghu_v8_phase_pitch_tuning/landing_time_trigger_50m_5s.json
```

这些运行产物按仓库导出策略默认不上传GitHub；源码、验收工具和文字结论上传。

## 10. 当前未完成项

1. 自动软着陆下沉率降到1 m/s以下；
2. 接地后轮刹车、落地状态和鸭翼-50°空气刹车的完整闭环；
3. 0.5/1/2 ms全步长收敛验收；
4. 真实试飞数据对V8气动参数的外部验证；
5. 低于50 m且不进入LAND的特殊低空任务目前会继续保持起飞权限，这是当前明确设计。

## 11. 推荐复现命令

```bash
cd /home/fly/PX4-Autopilot-NewAero
make px4_sitl gz_honghu_wing_150kg_v8
python3 Tools/honghu/check_honghu_v8.py
```

阶段切换专项：

```bash
python3 Tools/honghu/run_honghu_v8_dynamic_acceptance.py takeoff --step-size 0.002
python3 Tools/honghu/run_honghu_v8_dynamic_acceptance.py standard --step-size 0.002 --timeout 150 --no-assert
python3 Tools/honghu/run_honghu_v8_dynamic_acceptance.py landing --step-size 0.002 --no-assert
```

## 12. 100 kg与翔仪重新核对

4038在惯量插值前的源码默认参数下完成两次新的完整任务，均到LAND项18；离地空速约
40.25～40.37 m/s，起飞真值俯仰峰值约6.98～7.07°。翔仪与PX4全程水平差异RMS为
53.85～54.97 m，主要仍来自转弯圆弧；高度RMSE为2.61～2.69 m，俯仰RMSE为
1.74～1.80°。

当前平飞实际升降舵标准差已由旧日志约1.11°降至0.02～0.08°，高频舵面往复消失。
临时`FW_T_PTCH_DAMP=0.30`候选增大俯仰波动而被拒绝，4038保持0.15。4028和150 kg
生产模型未修改。仓库摘要见`HONGHU_V8_XIANGYI_RERUN_2026-07-30.md`，完整报告以
整体参考文档库同名文件为准。

之后仅使用Word表3已有的73/150 kg质量特性线性插值得到100 kg惯量；Word未给出的
参数不调整。上述两次完整任务保留为插值前闭环基线，不能替代当前插值惯量模型的
动态回归。

插值惯量模型随后完成701.856 s整条任务并到LAND项18；飞行功能检查通过，但该架次
缺少两路V8真值记录，所以自动状态仍为FAIL。紧随其后的短程起飞回归正式PASS，气动
与推进真值各记录1460个样本，验证了真实鸭翼角和诊断链。不得把两架次合称为单次
“全项PASS”，也没有因此调整Word缺失参数。
