# single_leg

这是 3D 单腿 computed-torque 控制器的正式标定实验。

对应模型：

```text
D:\Workspace\CodeWorkspace\dynamic\3D\simulate\single_leg\single_leg.slx
```

## 怎么运行

在 MATLAB 中执行：

```matlab
cd("D:\Workspace\CodeWorkspace")
addpath("calibration")
out = run_bayes_calibration("single_leg");
```

结果保存到：

```text
calibration/results/single_leg/<timestamp>/
```

## 优化参数

贝叶斯优化搜索这些 log 空间变量：

```text
log_bw_hz_h
log_bw_hz_k
log_zeta_h
log_zeta_k
```

它们会在 `experiment_config.m` 中映射为 computed-torque 控制增益：

```matlab
ctrl.wn = 2*pi*ctrl.bandwidthHz;
ctrl.Kp = diag(ctrl.wn.^2);
ctrl.Kd = diag(2 * ctrl.zeta .* ctrl.wn);
```

被标定的控制律是：

```matlab
v = ddqd + ctrl.Kd * (dqd - dq) + ctrl.Kp * (qd - q);
tau = M * v + C + G;
```

## 批量仿真配置

这个实验默认面向自动调参，而不是可视化观察：

```matlab
cfg.disableLogging = true;
cfg.disableSDI = true;
cfg.disableGraphics = true;
cfg.saveSignals = false;
cfg.maxLoggedPoints = 20000;
cfg.scopeDecimation = 1;
cfg.fastRestart = true;
cfg.plotFcn = [];
```

trial runner 会关闭大部分 Simulink 日志、SDI 记录、状态/输出/时间保存和 Scope 自动打开。当前模型还没有直接输出 scalar cost，所以 runner 会有限度保存 Scope 数据用于计算指标。

## 目标函数

`score_trial.m` 会把多个指标压成一个越小越好的标量：

```text
位置 RMS 误差
速度 RMS 误差
最终位置误差
力矩 RMS 比例
力矩饱和惩罚
```

权重在 `experiment_config.m` 的 `cfg.scoreWeights` 中设置。

## 长时间运行前检查

正式跑之前建议确认：

```matlab
cfg.stopTime = 4.0;
cfg.maxObjectiveEvaluations = 30;
cfg.variables = [
    optimizableVariable("log_bw_hz_h", log10([0.4, 5.0]))
    optimizableVariable("log_bw_hz_k", log10([0.4, 5.0]))
    optimizableVariable("log_zeta_h", log10([0.35, 2.0]))
    optimizableVariable("log_zeta_k", log10([0.35, 2.0]))
    ];
```

快速 smoke test 可以临时改成：

```matlab
cfg.maxObjectiveEvaluations = 3;
cfg.stopTime = 0.5;
```
