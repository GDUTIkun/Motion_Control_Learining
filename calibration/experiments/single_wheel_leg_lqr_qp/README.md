# single_wheel_leg_lqr_qp

这是为当前 2D 浮动基座单轮腿模型预留的长期实验入口。

对应模型：

```text
D:\Workspace\CodeWorkspace\dynamic\2D\simulate\single_wheel_leg\source.slx
```

这个目录以后用于放“如何批量运行这个模型”的稳定接口，例如：

```text
experiment_config.m
run_trial.m
score_trial.m
```

目前这个目录只是占位，暂时不放自动优化代码。当前扰动响应分析先放在：

```text
calibration/studies/2026_08_lqr_disturbance_response
```

## 未来用途

后续可以逐步加入：

```text
LQR Q/R 参数扫描
LQR 权重贝叶斯优化
QP 参数二阶段标定
扰动响应指标提取
接触敏感性测试入口
```

注意：模型、动力学和控制器实现仍然留在 `dynamic/2D/...`，这里不要复制模型本体。
