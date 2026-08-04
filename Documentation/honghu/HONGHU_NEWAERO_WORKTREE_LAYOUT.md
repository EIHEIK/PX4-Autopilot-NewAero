# 鸿鹄翼 NewAero 分支、worktree 与飞行数据布局

## 1. 目标

鸿鹄翼 V8 的 100 kg 与 150 kg 构型使用同一个 GitHub 仓库，但分别在独立
Git 分支和 worktree 中开发。这样可以隔离源码修改、构建产物、PX4 参数、
任务数据和 ULog，同时继续共享 Gazebo/PX4 接口、坐标转换、气动/推进基础
能力和分析工具。

## 2. 本地目录

```text
/home/fly/PX4-Autopilot-NewAero/          main（公共集成基线）
/home/fly/PX4-Autopilot-NewAero-100kg/    variant/honghu-v8-100kg
/home/fly/PX4-Autopilot-NewAero-150kg/    variant/honghu-v8-150kg

/home/fly/PX4-Autopilot-NewAero-flight-data/
├── 100kg/
├── 150kg/
└── baselines/
```

`PX4-Autopilot-NewAero-flight-data` 是worktree之外的长期归档目录，不属于
Git仓库。删除或重建worktree前，必须先把需要保留的飞行日志复制到这里。

## 3. 分支职责

- `main`：公共集成基线；保存经过两种重量构型共同验证的接口、坐标转换、
  Gazebo桥接、气动与发动机基础模型、日志接口、分析工具和文档。
- `variant/honghu-v8-100kg`：100 kg模型、参数、控制试验和验收结果。
- `variant/honghu-v8-150kg`：150 kg模型、参数、控制试验和验收结果。

型号专用改动先保留在对应型号分支。坐标系、执行器映射、气动插件、推进
插件等公共修复应先在`main`完成，然后分别同步到两个型号分支，并运行两套
回归检查。

## 4. 构建、参数和日志隔离

每个worktree使用自己的`build/px4_sitl_default`，因此以下内容天然隔离：

- PX4/Gazebo构建产物；
- `parameters.bson`和`parameters_backup.bson`；
- `dataman`任务数据库；
- `rootfs/log/`中的ULog；
- `analysis_outputs/`中的临时分析结果。

默认日志位置：

```text
/home/fly/PX4-Autopilot-NewAero-100kg/build/px4_sitl_default/rootfs/log/
/home/fly/PX4-Autopilot-NewAero-150kg/build/px4_sitl_default/rootfs/log/
```

## 5. 基线归档要求

每个稳定架次至少归档：

- 原始`.ulg`；
- 使用的任务`.plan`；
- PX4参数导出；
- Git提交号和分支名；
- 机型编号与质量构型；
- 验收结果或分析摘要。

推荐以日期和用途命名，例如：

```text
PX4-Autopilot-NewAero-flight-data/100kg/2026-08-04_classic-route/
PX4-Autopilot-NewAero-flight-data/150kg/2026-08-04_landing-baseline/
```

## 6. 当前过渡约束

当前4038仍从4028加载公共参数后再覆盖100 kg专用参数，模型生成工具也仍有
以150 kg为默认目标的历史结构。因此两个型号分支现阶段是开发环境隔离，
并不代表代码依赖已经彻底解耦。后续应将公共参数提取到独立公共配置，并使
4028和4038并列加载公共层，避免100 kg继续直接继承150 kg机型文件。
