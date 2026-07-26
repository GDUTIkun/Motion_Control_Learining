# 使用说明：single_leg PD/computed-torque 贝叶斯标定例程

## 1. 直接运行

在 MATLAB 中进入仓库根目录，然后运行：

```matlab
cd("D:\Workspace\CodeWorkspace")
addpath("calibration")
out = run_bayes_calibration("example_pd_single_leg");
```

运行结束后，结果默认保存在：

```text
calibration/results/example_pd_single_leg/<时间戳>/
```

主要输出文件：

- `best_params.mat`：最优参数。
- `bayes_results.mat`：完整 `bayesopt` 结果。
- `summary.csv`：每次 trial 的 objective 和参数记录。
- `objective_history.png`：优化过程曲线。
- `trials/trial_*.mat`：每次 trial 的参数、score、metrics、失败信息。

## 2. 这个例程在调什么

本例程使用现有模型：

```text
dynamic/3D/simulate/single_leg/single_leg.slx
```

贝叶斯优化搜索的是 log 空间参数：

- `log_bw_hz_h`
- `log_bw_hz_k`
- `log_zeta_h`
- `log_zeta_k`

这些参数会在 `experiment_config.m` 的 `map_params` 中映射为：

- `params.bandwidthHz`
- `params.zeta`

然后 `run_trial.m` 会进一步写入 Simulink 使用的 `ctrl`：

- `ctrl.bandwidthHz`
- `ctrl.wn`
- `ctrl.zeta`
- `ctrl.Kp`
- `ctrl.Kd`
- `ctrl.pdKp`
- `ctrl.pdKd`

## 3. 常用参数改哪里

主要改 `experiment_config.m`。

### 搜索范围

修改：

```matlab
cfg.variables = [
    optimizableVariable("log_bw_hz_h", log10([0.4, 5.0]))
    optimizableVariable("log_bw_hz_k", log10([0.4, 5.0]))
    optimizableVariable("log_zeta_h", log10([0.35, 2.0]))
    optimizableVariable("log_zeta_k", log10([0.35, 2.0]))
    ];
```

例如想扩大带宽搜索范围，就改 `log10([0.4, 5.0])`。

### 参数映射

修改同一个文件底部的：

```matlab
function params = map_params(params, cfg)
```

这里负责把 bayesopt 搜索变量转换成真实仿真参数。比如：

```matlab
params.bandwidthHz = [
    10 ^ params.log_bw_hz_h
    10 ^ params.log_bw_hz_k
    ];
```

以后做 LQR/NMPC/补偿器标定时，也建议在这里把 `log_q`、`log_r`、`log_weight` 等转换成真正的 Q/R/权重参数。

### 优化次数

修改：

```matlab
cfg.maxObjectiveEvaluations = 5;
```

调试时建议用 `5` 或 `10`。正式跑可以改成 `30`、`50` 或更多。

### 仿真时间

修改：

```matlab
cfg.stopTime = 4.0;
```

### 输出目录

默认：

```matlab
cfg.outputDir = "";
```

表示自动保存到 `calibration/results/...`。

如果想固定目录，可以改成：

```matlab
cfg.outputDir = "D:\Workspace\CodeWorkspace\calibration\results\my_run";
```

### 失败惩罚分

修改：

```matlab
cfg.penaltyScore = 1e9;
```

仿真报错、发散、输出缺失或 score 非有限值时，会返回这个大数，不会中断 `bayesopt`。

## 4. 内存和速度相关开关

仍然在 `experiment_config.m` 中修改：

```matlab
cfg.simulationMode = "normal";
cfg.fastRestart = false;
cfg.disableLogging = true;
cfg.disableSDI = true;
cfg.saveSignals = false;
```

建议默认保持：

- `cfg.disableLogging = true`：关闭 Simulink signal logging、状态/输出/时间保存等。
- `cfg.disableSDI = true`：关闭 SDI 记录。
- `cfg.saveSignals = false`：trial 文件不保存完整时序信号，只保存 metrics 和失败信息。

如果需要排查某次仿真，可以临时改成：

```matlab
cfg.saveSignals = true;
```

`cfg.fastRestart` 默认关闭，因为当前模型含 Interpreted MATLAB Function，Fast Restart 可能产生工作点警告。确认模型兼容后再改成：

```matlab
cfg.fastRestart = true;
```

## 5. 换成自己的实验

复制整个目录：

```text
calibration/experiments/example_pd_single_leg
```

例如复制为：

```text
calibration/experiments/my_lqr_single_leg
```

然后至少修改三个文件。

### experiment_config.m

改这些内容：

- `cfg.name`
- `cfg.variables`
- `map_params`
- `cfg.modelPath`
- `cfg.startupScript`
- `cfg.stopTime`
- `cfg.maxObjectiveEvaluations`
- `cfg.scoreWeights`
- 必要时修改 `cfg.scopeBlocks`、`cfg.scopeSaveNames`、`cfg.signalNames`

### run_trial.m

改这些内容：

- 如何把 `params` 写入模型、base workspace 或 `Simulink.SimulationInput`。
- 如何运行 `sim`。
- 如何从仿真输出中提取 metrics。
- 如何判断仿真失败、发散、输出缺失。

统一接口保持：

```matlab
result = run_trial(params, cfg)
```

建议 `result` 至少包含：

```matlab
result.success
result.metrics
result.errorMessage
result.elapsedTime
```

### score_trial.m

改 cost 公式：

```matlab
score = score_trial(result, params, cfg)
```

要求返回一个越小越好的标量。失败时返回：

```matlab
score = cfg.penaltyScore;
```

## 6. 推荐工作流程

1. 先单独确认原 Simulink 模型能正常运行。
2. 复制一个实验目录。
3. 在 `experiment_config.m` 定义搜索参数和映射。
4. 在 `run_trial.m` 写参数注入和仿真输出提取。
5. 在 `score_trial.m` 写 cost。
6. 先用很小次数测试：

```matlab
cfg.maxObjectiveEvaluations = 5;
```

7. 跑通后再增加优化次数。
8. 查看 `best_params.mat` 和 `summary.csv`。
