# 标定与数据分析工作区

这个目录用于长期存放仿真评估、数据提取、指标计算、画图、参数扫描和贝叶斯优化相关内容。

模型本体、动力学、控制器代码仍然放在对应的 `model/...` 目录中。`calibration` 只负责”怎么运行、怎么记录、怎么评价、怎么调参”。

## 目录结构

```text
calibration/
  core/          通用贝叶斯优化、trial 保存、参数转换等框架代码
  experiments/   每个模型/控制器对应的可复用实验入口
  configs/       共享配置，例如扰动工况、画图风格、指标权重
  studies/       具体分析课题，例如扰动响应、QR 扫描、接触敏感性
  data/          原始数据、处理后数据、外部数据
  results/       自动运行生成的结果
  figures/       筛选后值得保留的图
  reports/       阶段性结论和报告
  tools/         可复用的数据提取、画图、对比工具
  notebooks/     临时探索脚本或 Live Script
```

## 基本边界

推荐一直保持这个边界：

```text
model       = 被测系统本体
calibration = 实验、分析、标定平台
```

例如当前 2D 单轮腿倒立摆模型在：

```text
D:\Workspace\CodeWorkspace\model\simulate\base_with_wheel_leg
```

它的批量扰动测试、指标统计、画图和后续优化应该放在：

```text
calibration/studies/...
calibration/experiments/single_wheel_leg_lqr_qp
calibration/results/...
```

## 推荐工作流

1. 如果只是研究一个具体问题，先在 `studies/` 下建一个 study。
2. 如果某个模型需要长期反复运行，再在 `experiments/` 下建实验入口。
3. 原始仿真输出放 `results/` 或 `data/raw/`。
4. 提取后的指标表放 `data/processed/`。
5. 临时探索放 `notebooks/`，可复用函数沉淀到 `tools/`。
6. 最终要引用或写报告的图放 `figures/`。

## 当前重点

当前建议优先使用：

```text
calibration/studies/2026_08_lqr_disturbance_response
```

它用于评估 2D 浮动基座 LQR + QP 单轮腿闭环在小扰动下的响应。
