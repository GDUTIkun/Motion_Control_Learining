# single_wheel_leg_qp

这是旧的 3D 单轮腿 QP/WBC 控制器标定实验。

对应模型：

```text
D:\Workspace\CodeWorkspace\dynamic\3D\simulate\single_wheel_leg\source.slx
```

注意：这个实验不是当前 2D 浮动基座 LQR + QP 模型的入口。当前 2D 模型的扰动响应分析在：

```text
calibration/studies/2026_08_lqr_disturbance_response
```

## 怎么运行

在 MATLAB 中执行：

```matlab
cd("D:\Workspace\CodeWorkspace")
addpath("calibration")
out = run_bayes_calibration("single_wheel_leg_qp");
```

结果保存到：

```text
calibration/results/single_wheel_leg_qp/<timestamp>/
```

## 优化参数

贝叶斯优化搜索这些 log 空间变量：

```text
log_bw_hz_h
log_bw_hz_k
log_bw_hz_w
log_zeta
log_constraint_gain
log_qp_wtau
log_qp_wfc
```

它们会映射到 QP 控制器参数：

```matlab
ctrl.wn = 2*pi*params.bandwidthHz;
ctrl.zeta = params.zeta;
ctrl.Kp = diag(ctrl.wn.^2);
ctrl.Kd = diag(2 * ctrl.zeta .* ctrl.wn);
ctrl.constraintVelocityGain = params.constraintVelocityGain;
ctrl.qpWtau = params.qpWtau;
ctrl.qpWFc = params.qpWFc;
```

第一阶段固定 `hip` 位置 PD 参数，只标定 QP 相关参数。等 QP 层稳定后，再单独调 `hip` 位置控制。

## 批量运行配置

默认配置参考 `single_leg` 的正式标定流程：

```matlab
cfg.useParallel = false;
cfg.disableLogging = true;
cfg.disableSDI = true;
cfg.disableGraphics = true;
cfg.saveSignals = false;
cfg.maxLoggedPoints = 15000;
cfg.scopeDecimation = 2;
cfg.fastRestart = true;
cfg.plotFcn = [];
```

只保存六个 Scope 的有界数据：

```text
qh, qk, qw
dqh, dqk, dqw
```

runner 不调用 `save_system`。它只会临时把顶层 controller block 设置为 `controller_qp`，trial 结束后恢复。

## 快速测试

长时间运行前，建议先把配置临时改小：

```matlab
cfg.maxObjectiveEvaluations = 3;
cfg.stopTime = 1.0;
```

正式运行可以改为：

```matlab
cfg.maxObjectiveEvaluations = 30;
cfg.stopTime = 14.0;
```
