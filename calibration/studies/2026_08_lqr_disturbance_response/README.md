# 2026-08 LQR 扰动扫描分析

这个 study 用来扫描当前 2D 浮动基座单轮腿模型的稳定工作范围。

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
5. 调用 configure_model(false)，只临时配置模型，不保存 source.slx
6. 关闭可视化和不必要日志
7. 标记并抓取核心信号
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
初始 pitch = [0, 1, 2, 3, 5] deg
脉冲力     = [0, 2, 5, 10] N
```

也就是 5 x 4 = 20 个工况。

如果想改扫描范围，直接修改：

```matlab
pitchDegList = [0, 1, 2, 3, 5];
pulseAmplitudeList = [0, 2, 5, 10];
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
pulseDelay = 7.0;
pulsePeriod = 10.0;
pulseWidthPercent = 5.0;
```

所以默认脉冲窗口是：

```text
7.0 s 到 7.5 s
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

当前通过 SDI 标记并抓取四路核心信号：

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
max |theta| <= 10 deg
final |theta| <= 1 deg
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

但它会保留必要的 SDI 信号记录，因为当前数据提取依赖四路被标记信号。
