# 2026-08 LQR 扰动扫描分析

这个 study 用来扫描当前 2D 浮动基座单轮腿模型的稳定工作范围。

## 当前状态更新（2026-08-06）

后续抓取数据、实验设计和数据分析都在 `calibration` 工作区完成；
`dynamic/2D/simulate/single_wheel_leg` 只保留模型本体、动力学和控制器代码。

当前讨论决定先进入阶段 A：

```text
不改控制器，先定义并验证学习模型的接受指标和脉冲鲁棒性。
目标工作范围：theta0 = +/-10 deg, dtheta0 = 0。
鲁棒性输入：2.0 s 时加入 0.5 s 水平脉冲。
轮子保持在 hip 正下方不是硬控制目标，只是阶段 1 IK 参考生成方式。
```

本 study 已切到阶段 A 轻量入口：

```text
theta0List = [-10, 0, 10] deg
pulseAmplitudeList = [0, 5, 10] N
pulseWindow = [2.0, 2.5] s
stopTime = 5 s
configure_discrete_controller_timing(false)
stage-1 floating_base_leg_reference 指标口径
```

为减少批量运行耗时，`run_cases.m` 现在直接从 `simOut.logsout` 提取四路核心信号，
不再通过 SDI streaming 抓取数据。正式扫描前仍建议先跑 1 个 case 做 smoke test。

Smoke test（2026-08-06）：

```text
case: pitch_-10deg_pulse_0N
resultDir: calibration/results/studies/2026_08_lqr_disturbance_response/20260806_193715
stable: true
failureReason: ok
maxAbsThetaDeg: 10
finalThetaDeg: 0.00967
settlingTimeTheta: 1.80 s
tauSaturationRatio: 0.069
elapsedSeconds: 42.18 s
```

The updated `logsout` extraction path worked on this smoke case.

Pulse smoke test（2026-08-06）：

```text
case: pitch_-10deg_pulse_5N
resultDir: calibration/results/studies/2026_08_lqr_disturbance_response/20260806_194726
stable: true
failureReason: ok
maxAbsThetaDeg: 10
finalThetaDeg: 0.0187
pulseMaxAbsThetaDeg: 5.36
pulseMaxAbsTau: 5.96
pulseMaxAbsULqr: 63.47
elapsedSeconds: 40.98 s
```

Bugfix: pulse-window metrics now use each signal's own time vector. This avoids
logical-index length mismatches when `base`, `tau`, and `uLqr` have different
logged sample counts.

High-disturbance smoke（2026-08-06）：

```text
case: pitch_10deg_pulse_10N
resultDir: calibration/results/studies/2026_08_lqr_disturbance_response/20260806_195452
simulation completed: yes
stable: false
failureReason: max theta too large; final theta not settled; final dtheta not settled; tau near saturation; x drift too large
```

Bugfix: stage-1 leg reference now projects the wheel-center target into the
reachable two-link workspace, and IK velocity/acceleration solves use damped
least squares near singularity. This prevents Scope/reference generation from
producing invalid outputs when a failed case drives the hip/wheel geometry
outside the nominal IK range.

Full stage-A result（2026-08-06）：

```text
resultDir: calibration/results/studies/2026_08_lqr_disturbance_response/20260806_195701
report: calibration/reports/2026_08_lqr_disturbance_response_summary.md
stable: 6 / 9
passed: all 0 N and 5 N pulse cases for theta0 = [-10, 0, 10] deg
failed: all 10 N pulse cases
```

Decision:

```text
Stop scanning here. Do not refine pulseAmplitude = [6, 7, 8, 9] N for now.
```

Next work should interpret representative pass/fail cases and then tune LQR/QP
parameters against this fixed 9-case baseline. Do not continue enlarging the
stage-A sweep yet. Horizontal and vertical motion tests come later, after
stationary robustness is acceptable.

被测系统：

```text
D:\Workspace\CodeWorkspace\dynamic\2D\simulate\single_wheel_leg\source.slx
```

分析对象：

```text
上层：浮动基座 LQR
下层：轮腿 QP 控制器
植物：Simscape 三自由度浮动基座 + 单轮腿
```

## 目标

这不是找最优参数，而是先回答：

```text
当前默认 LQR + QP 能承受多大的初始 pitch 扰动？
能承受多大的外力脉冲？
失效时先出现什么：theta 回不来、x 漂太远，还是 tau 接近饱和？
```

## 怎么运行

在 MATLAB 中执行：

```matlab
cd("D:\Workspace\CodeWorkspace\calibration\studies\2026_08_lqr_disturbance_response")
out = run_cases;
plot_responses(out);
```

`run_cases` 会自动：

```text
1. 进入 dynamic/2D/simulate/single_wheel_leg
2. 运行 startup
3. 设置每个 case 的 base.x0 初始状态
4. 设置 Pulse Generator 的脉冲变量
5. 调用 configure_discrete_controller_timing(false)，只临时配置模型，不保存 source.slx
6. 关闭可视化和不必要日志
7. 从 logsout 抓取核心信号
8. 运行仿真
9. 计算指标并保存结果
```

## 默认扫描范围

扫描定义在：

```text
disturbance_cases.m
```

默认扫描：

```text
初始 pitch = [-10, 0, 10] deg
脉冲力     = [0, 5, 10] N
脉冲窗口   = 2.0 s 到 2.5 s
仿真时间   = 5 s
```

也就是 3 x 3 = 9 个工况。

如果想改扫描范围，直接修改：

```matlab
pitchDegList = [-10, 0, 10];
pulseAmplitudeList = [0, 5, 10];
```

## 初始状态怎么定义

每个 case 的初始状态是：

```matlab
x0 = [xB; zB; thetaB; dxB; dzB; dthetaB]
```

默认只扫描 `thetaB`：

```matlab
c.x0 = [0; 0; deg2rad(pitchDeg); 0; 0; 0];
```

如果以后要扫初始速度或初始位置，也在 `disturbance_cases.m` 里改 `x0`。

## 脉冲怎么定义

模型里使用 Simulink 的 Pulse Generator 模块：

```text
source/PD_only/Pulse Generator
```

`run_cases` 会把它的参数设置成 base workspace 变量：

```matlab
disturbancePulseAmplitude
disturbancePulsePeriod
disturbancePulseWidth
disturbancePulseDelay
```

默认值来自每个 case：

```matlab
pulseDelay = 2.0;
pulsePeriod = 10.0;
pulseWidthPercent = 5.0;
```

所以默认脉冲窗口是：

```text
2.0 s 到 2.5 s
```

如果 `pulseAmplitudeN = 0`，就是无脉冲工况。

## 输出结果在哪里

每次运行会新建一个带时间戳的结果目录：

```text
D:\Workspace\CodeWorkspace\calibration\results\studies\2026_08_lqr_disturbance_response\<timestamp>
```

里面会保存：

```text
每个 case 一个 .mat 文件
study_results.mat
summary.csv
```

`study_results.mat` 里保存完整 `out` 结构。`summary.csv` 是所有工况的指标表。

## 抓取哪些信号

当前通过 `logsout` 抓取四路核心信号：

```text
source/PD_only/Mux                         基座状态输入
source/PD_only/Mux1                        QP 16 维输入
source/PD_only/Interpreted MATLAB Function LQR 输出
source/Interpreted MATLAB Function         QP 输出力矩
```

保存后的主要信号含义：

```text
X      = [xB, zB, thetaB, dxB, dzB, dthetaB]
uLqr   = [FHx_ext, FHz_ext, MBy_des]
tau    = [tau_h, tau_k, tau_w]
qRel   = [qh, qk, qw]
dqRel  = [dqh, dqk, dqw]
qRef   = [qh_des, qk_des, qw_des]
dqRef  = [dqh_des, dqk_des, dqw_des]
```

## summary.csv 怎么看

关键列：

```text
initialPitchDeg      初始 pitch 扰动，单位 deg
pulseAmplitudeN      脉冲力大小，单位 N
stable               是否满足当前稳定判据
failureReason        不稳定原因
maxAbsThetaDeg       最大 pitch 角，单位 deg
finalThetaDeg        最终 pitch 角，单位 deg
finalX               最终基座 x 位移
settlingTimeTheta    theta 回到 0.5 deg 误差带后的时间
tauSaturationRatio   最大力矩占力矩上限比例
legPositionRms       qh/qk 相对期望的 RMS 位置误差
legVelocityRms       dqh/dqk 相对期望的 RMS 速度误差
pulseMaxAbsThetaDeg  脉冲窗口内最大 pitch 角
pulseMaxAbsTau       脉冲窗口内最大关节力矩
pulseMaxAbsULqr      脉冲窗口内最大 LQR 输出
```

## 当前稳定判据

`stable` 的第一版判据在 `run_cases.m` 里：

```text
max |theta| <= 15 deg
final |theta| <= 2 deg
final |dtheta| <= 0.1 rad/s
tauSaturationRatio <= 0.95
max |x| <= 0.5 m
```

这个判据只是为了找大概工作范围，不是最终控制指标。后面可以根据实验目标收紧或放宽。

## 分析流程

1. 先看 `summary.csv`，找出 `stable = false` 的工况。
2. 按 `initialPitchDeg` 和 `pulseAmplitudeN` 排序，找稳定/不稳定边界。
3. 如果 `failureReason` 是 `final theta not settled`，优先考虑 LQR 的 Q/R 或积分。
4. 如果是 `tau near saturation`，说明执行器或 QP 层先到极限。
5. 如果是 `x drift too large`，说明靠滚动恢复姿态的代价太大。
6. 再用 `plot_responses(out)` 看边界附近的具体时域响应。

## 只跑少量工况测试

如果只是检查脚本是否能跑通，可以在 MATLAB 中手动构造少量 case：

```matlab
cases = disturbance_cases();
cases = cases(1:2);
out = run_cases(cases);
```

或者改 `disturbance_cases.m` 里的扫描列表。

## 可视化配置

`run_cases` 会关闭 Mechanics Explorer 自动打开、Scope 自动打开、Scope workspace 保存、Simscape logging、普通 output/state/time 保存等，以便批量跑得更快。

数据提取现在依赖 `simOut.logsout` 中已有的四路核心信号，不再依赖 SDI streaming。
