# 单轮腿 LQR Q/R 贝叶斯优化

这个实验用于当前 2D 浮动基座单轮腿模型的第一轮参数优化。

当前定位：

```text
只调上层 floating-base LQR 的 Q/R。
固定下层 QP、接触参数、力矩上限和模型结构。
目标是在静止站立抗扰阶段提高鲁棒性。
```

基线结果：

```text
calibration/results/studies/2026_08_lqr_disturbance_response/20260806_195701
```

基线结论：

```text
theta0 = [-10, 0, 10] deg
pulse = [0, 5, 10] N

0 N / 5 N 全部通过
10 N 全部失败
```

代表机制分析：

```text
calibration/reports/2026_08_pass_fail_mechanism.md
```

## 优化变量

使用倍率搜索，不直接搜索绝对值：

```text
log_s_qtheta
log_s_qdtheta
log_s_qx
log_s_qdx
log_s_rfhx
log_s_rfz
log_s_rmb
```

含义：

```text
Q = diag([x, z, theta, dx, dz, dtheta])
R = diag([FHx, FHz, MBy])
```

固定不调：

```text
Q_z, Q_dz
QP 参数
接触参数
tauMax
```

为什么加入 `R_Fz`：

```text
代表失败工况 pitch_10deg_pulse_10N 中，最早出现的上层饱和在 FHz_ext。
```

## 训练工况

训练工况定义在：

```text
qr_training_cases.m
```

当前使用 5 个代表工况：

```text
pitch_0deg_pulse_0N
pitch_10deg_pulse_5N
pitch_-10deg_pulse_5N
pitch_0deg_pulse_10N
pitch_10deg_pulse_10N
```

每个工况：

```text
stopTime = 5 s
pulseWindow = [2.0, 2.5] s
```

完整 9 工况不直接放入每个 Bayesian trial，避免一次优化过慢。优化后必须回到完整 9 工况验证。

## 运行顺序

先跑一个 smoke test：

```matlab
cd("D:\Workspace\CodeWorkspace\calibration\experiments\single_wheel_leg_lqr_qr")
score = run_smoke_test;
```

再跑默认 Q/R 的训练集检查：

```matlab
score = run_default_qr_check;
```

确认默认训练集能正常完成后，再启动贝叶斯优化：

```matlab
cd("D:\Workspace\CodeWorkspace\calibration")
out = run_bayes_calibration("single_wheel_leg_lqr_qr");
```

结果会保存到：

```text
D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr\<timestamp>
```

## 评分目标

评分函数惩罚：

```text
theta RMS
final theta
max theta
dtheta RMS
final dtheta
x drift
tau saturation
uLqr saturation
leg q/dq tracking RMS
unstable penalty
```

稳定判据与阶段 A 保持一致：

```text
max |theta| <= 15 deg
final |theta| <= 2 deg
final |dtheta| <= 0.1 rad/s
tauSaturationRatio <= 0.95
max |x| <= 0.5 m
```

## 调完后

拿到 `best_params.mat` 后，不要直接认为优化完成。

必须将最优 Q/R 代回完整 9 工况验证：

```text
theta0 = [-10, 0, 10] deg
pulse = [0, 5, 10] N
```

对比基线：

```text
stable 数量是否增加
10 N 工况是否改善
finalThetaDeg / finalDtheta 是否下降
x drift 是否变小
tauSaturationRatio 是否远离 1
5 N 已通过工况是否被破坏
```
