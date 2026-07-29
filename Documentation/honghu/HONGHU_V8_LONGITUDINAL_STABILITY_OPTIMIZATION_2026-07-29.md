# 鸿鹄翼 V8 纵向稳定性优化摘要（2026-07-29）

权威完整报告位于：

```text
/home/fly/px4_reference_docs/current/HONGHU_V8_LONGITUDINAL_STABILITY_OPTIMIZATION_2026-07-29.md
```

当前源码结论：

- 100 kg隔离机型4038：`FW_PR_D=0.03`、`TRIM_PITCH=-0.09`、
  `FW_T_PTCH_DAMP=0.15`；
- 150 kg生产机型4028：俯仰率内环保持
  `P/I/D/FF=0.40/0.04/0.10/0.75`，`FW_T_PTCH_DAMP=0.30`；
- 150 kg继续使用 `FW_T_SPDWEIGHT=1.0`、`FW_T_I_GAIN_PIT=0.05`、
  `FW_T_ALT_TC=3.5`；
- 组合候选 `FW_T_SPDWEIGHT=0.7 + FW_T_PTCH_DAMP=0.30`因在特定持续转弯
  放大高度和空速波动而被拒绝。

150 kg单阻尼完整任务到达LAND项18。按任务序号逐段对齐后，相对原完整任务，
高度、俯仰和空速波动分别下降约10.4%、6.3%和4.6%；主要持续转弯段分别下降
14.9%、12.8%和9.1%。起飞最大真值俯仰8.25°。

100 kg完整优化任务同样到达LAND项18，俯仰率积分器由-0.12饱和释放到约-0.040，
起飞最大真值俯仰6.70°。积分器没有关闭。

4038源码默认短程回归也已PASS，并由ULog确认实际加载
`FW_PR_D=0.03`、`TRIM_PITCH=-0.09`、`FW_T_PTCH_DAMP=0.15`。

结果目录：

```text
analysis_outputs/honghu_v8_longitudinal_control_diagnosis/
analysis_outputs/honghu_v8_longitudinal_tuning/
```
