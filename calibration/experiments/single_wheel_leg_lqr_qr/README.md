# 单轮腿 LQR Q/R 贝叶斯优化

这个实验只调上层浮动基座 LQR 的 Q/R，固定下层 QP、接触参数和力矩上限。

## 运行

先在已经打开的 MATLAB 中跑一个短冒烟测试：

```matlab
cd("D:\Workspace\CodeWorkspace\calibration\experiments\single_wheel_leg_lqr_qr")
score = run_smoke_test;
```

再跑默认 Q/R 的训练集检查：

```matlab
score = run_default_qr_check;
```

如果默认 Q/R 检查里的训练工况都不稳定，先不要启动贝叶斯优化，要回到 Simulink 模型、初始条件、求解器和接口检查。

手动设置初始 pitch 时，不要直接改 `base.x0(3)`。使用：

```matlab
set_initial_pitch(deg2rad(2));
configure_model(false);
```

这个函数会同步更新 `base.x0`、`leg.q0`、`leg.dq0` 和 `baseLqr`。

确认能跑通后，再启动完整贝叶斯优化：

在 MATLAB 中执行：

```matlab
cd("D:\Workspace\CodeWorkspace\calibration")
out = run_bayes_calibration("single_wheel_leg_lqr_qr");
```

结果会保存到：

```text
D:\Workspace\CodeWorkspace\calibration\results\single_wheel_leg_lqr_qr\<timestamp>
```

主要看：

```text
summary.csv
best_params.mat
bayes_results.mat
trials\trial_*.mat
```

## 优化变量

使用倍率搜索，不直接搜索绝对值：

```text
log_s_qtheta
log_s_qdtheta
log_s_qx
log_s_qdx
log_s_rfhx
log_s_rmb
```

每个变量范围是 `[-0.5, 0.5]`，也就是默认参数的约 `0.316x ~ 3.16x`。

第一轮优化会把默认 Q/R 作为初始点，避免优化器只在随机失败参数里搜索。

固定不调：

```text
Q_z, Q_dz, R_Fz
QP 参数
接触参数
tauMax
```

## 训练工况

训练工况定义在：

```text
qr_training_cases.m
```

当前使用：

```text
0 deg, 0 N
2 deg, 0 N
3 deg, 5 N
3 deg, 10 N
5 deg, 0 N
```

第一轮先不把已知失败边界 `5 deg, 2 N` 放进训练集。先把稳定区内响应调好，再用完整扫描集验证边界有没有改善。

## 目标函数

目标函数主要惩罚：

```text
theta RMS
最终 theta
最大 theta
x 漂移
力矩使用比例
LQR 输出使用比例
腿部 q/dq 跟踪误差
失稳惩罚
```

如果某个训练工况不满足稳定判据，会额外加大惩罚。

## 调完后做什么

拿到 `best_params.mat` 后，不要直接认为模型调好了。

下一步要把最优 Q/R 代回完整扫描集复测，比较：

```text
stable 数量是否增加
5 deg 边界是否改善
finalThetaDeg 是否变小
finalX 是否明显变大
tauSaturationRatio 是否接近 1
qh/qk 跟踪是否恶化
```
