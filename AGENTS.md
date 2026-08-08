# CodeWorkspace 项目级目录边界（所有 agent 必须遵守）

这是一个控制/仿真仓库。写**任何新文件**之前，先判定它属于哪一类，放到正确的边界内。核心划分：

```text
model       = 被测系统本体（模型、动力学、控制器代码、Simulink .slx）
calibration = 实验、分析、标定平台（怎么跑、怎么记、怎么评价、怎么调参）
tools       = 第三方库（acados/casadi），禁止放项目代码
figures     = 顶层图床（模型示意图/原理图等归档图）
```

## 第一判定：仿真 vs 分析

- **仿真类**（动力学/运动学/IK/状态方程、LQR/QP 控制器、Simulink 模型、模型侧 configure/startup 脚本）
  → `model/` 下。模型代码放 `model/code/`，Simulink 模型放 `model/simulate/<model>/`。
- **分析类**（批量工况运行、信号提取、指标计算、画图、参数扫描、贝叶斯优化、结果汇总、报告）
  → `calibration/` 下。

## calibration 内部分流

| 内容                                                            | 位置                                     |
| --------------------------------------------------------------- | ---------------------------------------- |
| 通用框架代码（贝叶斯优化、trial 保存、参数转换）                | `calibration/core/`                    |
| 与某模型强绑定的运行入口                                        | `calibration/experiments/<name>/`      |
| 具体分析课题（run_cases / plot_*.m / extract_*.m / notes.md） | `calibration/studies/<YYYY_MM_topic>/` |
| 原始仿真输出                                                    | `calibration/data/raw/`                |
| 提取后的指标表（csv）                                           | `calibration/data/processed/`          |
| 批量运行结果目录                                                | `calibration/results/<study>/`         |
| 值得保留的图                                                    | `calibration/figures/studies/<study>/` |
| 阶段性结论/报告                                                 | `calibration/reports/YYYY_MM_*.md`     |
| 被多个 study 复用的函数                                         | `calibration/tools/`                   |
| 临时探索                                                        | `calibration/notebooks/`               |

流转规则：只服务一个 study 的脚本先放 `studies/<study>/`；多个 study 复用再升到 `tools/`；强绑定模型的运行逻辑放 `experiments/`。完整约定见 `calibration/README.md`。

## 反面清单（常见放错）

- 把分析脚本/数据表/结果图放进 `model/` → 应移入 `calibration/`
- 把动力学/控制器源码放进 `calibration/`（`experiments/` 里的运行包装除外）→ 应移入 `model/`
- 在 `tools/` 放项目代码 → 禁止，`tools/` 只放第三方库
- 根目录散落源码（`source.slx`/`source.slxc` 除外）→ 应移入对应子目录

`slprj/`、`*.slxc`、`.tmp_*` 是 Simulink 构建产物，已 gitignore，不属于本项目边界管理范围。
