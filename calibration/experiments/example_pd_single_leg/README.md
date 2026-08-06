# example_pd_single_leg

这是一个贝叶斯优化示例实验，用来演示 `calibration` 平台的基本用法。

对应模型：

```text
D:\Workspace\CodeWorkspace\dynamic\3D\simulate\single_leg\single_leg.slx
```

这个实验主要用于学习平台流程，不建议作为当前 2D 单轮腿 LQR/QP 的分析入口。

## 怎么运行

在 MATLAB 中执行：

```matlab
cd("D:\Workspace\CodeWorkspace")
addpath("calibration")
out = run_bayes_calibration("example_pd_single_leg");
```

运行结束后，结果默认保存到：

```text
calibration/results/example_pd_single_leg/<timestamp>/
```

常见输出文件：

```text
best_params.mat        最优参数
bayes_results.mat      完整 bayesopt 结果
summary.csv            每次 trial 的 objective 和参数记录
objective_history.png  优化过程曲线
trials/trial_*.mat     每次 trial 的参数、score、metrics、错误信息
```

## 这个实验在调什么

贝叶斯优化搜索这些 log 空间变量：

```text
log_bw_hz_h
log_bw_hz_k
log_zeta_h
log_zeta_k
```

这些变量在 `experiment_config.m` 的 `map_params` 中映射为：

```text
params.bandwidthHz
params.zeta
```

然后 `run_trial.m` 会把参数写入 Simulink 使用的 `ctrl`：

```text
ctrl.bandwidthHz
ctrl.wn
ctrl.zeta
ctrl.Kp
ctrl.Kd
ctrl.pdKp
ctrl.pdKd
```

## 常改位置

主要改：

```text
experiment_config.m
```

常见修改项：

```text
cfg.variables                 搜索变量和范围
map_params                    参数映射
cfg.maxObjectiveEvaluations   优化次数
cfg.stopTime                  仿真时间
cfg.outputDir                 输出目录
cfg.penaltyScore              失败惩罚分数
cfg.scoreWeights              指标权重
```

调试时建议先用很小次数：

```matlab
cfg.maxObjectiveEvaluations = 5;
cfg.stopTime = 0.5;
```

确认流程跑通后，再增加优化次数。

## 复制成新实验

如果要创建自己的贝叶斯优化实验，可以复制整个目录：

```text
calibration/experiments/example_pd_single_leg
```

然后至少修改：

```text
experiment_config.m
run_trial.m
score_trial.m
```

统一接口保持：

```matlab
result = run_trial(params, cfg)
score = score_trial(result, params, cfg)
```

失败时 `score_trial` 应返回：

```matlab
score = cfg.penaltyScore;
```
