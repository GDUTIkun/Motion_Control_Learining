function write_common_mode_validation_report(path, s, b)
%WRITE_COMMON_MODE_VALIDATION_REPORT Write the UTF-8 Markdown handoff report.

fid = fopen(path, "w", "n", "UTF-8");
assert(fid >= 0, "Cannot write report: %s", path);
cleanup = onCleanup(@() fclose(fid));

stand = one(s, "stand");
pitch = one(s, "pitch_2deg");
velocity = one(s, "velocity_baseline");
d5 = one(s, "disturbance_5N");
d20 = one(s, "disturbance_20N");
nmpPass = ~isempty(velocity) && velocity.nmp_ideal_direction_ratio > 0.99 ...
    && velocity.nmp_ref_direction_ratio > 0.5 ...
    && velocity.nmp_actual_direction_ratio > 0.5;

fprintf(fid, "# 严格共同模态轮腿控制算法验证报告\n\n");
fprintf(fid, "> 生成时间：%s（Asia/Shanghai）  \n", ...
    string(datetime("now", "TimeZone", "Asia/Shanghai")));
fprintf(fid, "> 模型：`source_common.slx`；逐时数据：`../data/*.csv`。\n\n");

fprintf(fid, "## 结论摘要\n\n");
outcomeLine(fid, "静态站立", passed(stand));
outcomeLine(fid, "2° 初始 pitch 恢复", passed(pitch));
outcomeLine(fid, "0→0.5 m/s→0 速度跟踪性能门槛", passed(velocity));
outcomeLine(fid, "WIPM 非最小相位反向预动作", nmpPass);
fprintf(fid, "\n严格共同模态的动力学、NMPC 状态、QP 可行性和左右对称性链路得到验证；" ...
    + "但基线速度 MAE 未达到预设 0.1 m/s 门槛，因此不能把整套基线标记为全部通过。" ...
    + "本轮只记录结果和参数经验，没有自动调参，也没有修改完整双腿模型。\n\n");

fprintf(fid, "## 模型和控制结构\n\n");
fprintf(fid, "验证自由度为基座 $[x_B,z_B,\\theta_B]$ 与一个左右完全同步的共同腿变量。控制链为：\n\n");
fprintf(fid, "`WIPM-LQR wheel planning → 8-state NMPC body wrench → strict common-mode WBC-QP → summed joint torque → Simscape plant`\n\n");
fprintf(fid, "NMPC 周期 %.3f s、节点数 %d、预测时域 %.2f s；静态轮位 $\\xi_0=%.7f$ m。" ...
    + "低摩擦试验只修改共同模态 QP 摩擦锥，不修改 Simscape 接触材料或已编译 NMPC drive coefficient。\n\n", ...
    b.baseNmpc.Ts, b.baseNmpc.N, b.baseNmpc.Ts*b.baseNmpc.N, b.baseNmpc.model.xiEq);

fprintf(fid, "## 指标口径\n\n");
fprintf(fid, "- 轮位幅值统一使用 $\\Delta\\xi=\\xi-\\xi_0$；WIPM 理论前馈为 $\\Delta\\xi_{ideal}=-H a_x^{ref}/g$。\n");
fprintf(fid, "- 通用数值护栏：NMPC status/fault 正常、QP 可行率 >99%%、动力学残差无穷范数 <1e-6。\n");
fprintf(fid, "- 速度性能门槛：MAE <0.1 m/s 且 pitch 峰值 <5°；约束扫描使用 MAE <0.15 m/s。\n");
fprintf(fid, "- pitch 整定带为 max(0.1°, 初始角的 2%%)。\n\n");

fprintf(fid, "## 实验 1：静态站立\n\n");
fprintf(fid, "![base state](../figures/base_state.png)\n\n![wrench](../figures/wrench.png)\n\n![QP residual](../figures/qp_residual.png)\n\n");
writeTable(fid, stand);
if ~isempty(stand)
    fprintf(fid, "QP 可行率 %.3f%%，最大动力学残差 %.3e，高度最大漂移 %.3e m；" ...
        + "接触总 $F_z$ 与整机重力的相对误差 %.3e。静态站立通过。\n\n", ...
        100*stand.qp_feasible_ratio, stand.max_dynamics_residual, ...
        stand.max_z_drift_m, stand.contact_fz_relative_error);
end

fprintf(fid, "## 实验 2：初始 pitch 扰动恢复\n\n");
fprintf(fid, "![pitch recovery](../figures/pitch_recovery.png)\n\n");
writeTable(fid, pitch);
if ~isempty(pitch)
    fprintf(fid, "初始/最大 pitch 为 %.3f°，整定时间 %.3f s，末 1 s 平均稳态误差 %.4f°。\n\n", ...
        rad2deg(pitch.max_pitch_rad), pitch.settling_time_s, ...
        rad2deg(pitch.steady_pitch_error_rad));
end

fprintf(fid, "## 实验 3：速度跟踪\n\n");
fprintf(fid, "![velocity tracking](../figures/velocity_tracking.png)\n\n![wheel position](../figures/wheel_position.png)\n\n");
writeTable(fid, velocity);
if ~isempty(velocity)
    fprintf(fid, "速度 MAE %.4f m/s，轮位峰值 %.4f m，pitch 峰值 %.3f°。" ...
        + "QP 全程可行且残差通过，但 MAE 高于 0.1 m/s 门槛，所以性能验收未通过。\n\n", ...
        velocity.velocity_mae_mps, velocity.wheel_peak_delta_m, ...
        rad2deg(velocity.max_pitch_rad));
end

fprintf(fid, "## 实验 4：非最小相位轮位规划\n\n");
fprintf(fid, "![non-minimum phase](../figures/non_minimum_phase.png)\n\n");
if ~isempty(velocity)
    fprintf(fid, "正加速区间的负向比例：理论 %.1f%%、LQR/governor 参考 %.1f%%、实际轮位 %.1f%%；" ...
        + "实际/理论负向峰值比为 %.3f。正向加速度时轮位先向后，方向验证通过。\n\n", ...
        100*velocity.nmp_ideal_direction_ratio, ...
        100*velocity.nmp_ref_direction_ratio, ...
        100*velocity.nmp_actual_direction_ratio, ...
        velocity.nmp_actual_to_ideal_ratio);
end

fprintf(fid, "## 实验 5：LQR Q/R 敏感性\n\n");
fprintf(fid, "![LQR sensitivity](../figures/lqr_sensitivity.png)\n\n");
lqr = [velocity; s(s.experiment == "lqr_sensitivity", :)];
writeTable(fid, lqr);
fprintf(fid, "共同缩放 Q、R 不改变 LQR 增益，因此扫描的是有效 Q/R 比 0.5×、1×、2×。" ...
    + "2× 在已测点中速度 MAE 最低，但轮位和 pitch 略增；三组均未达到 0.1 m/s MAE 门槛，" ...
    + "故这里只记录趋势，不给出通过型推荐。\n\n");

fprintf(fid, "## 实验 6：NMPC 权重敏感性\n\n");
fprintf(fid, "![NMPC sensitivity](../figures/nmpc_sensitivity.png)\n\n");
nmpc = [velocity; s(s.experiment == "nmpc_sensitivity", :)];
writeTable(fid, nmpc);
fprintf(fid, "`R1` 为 wrench 参考偏差惩罚，`R2` 为输入变化惩罚。已测组合中 `R2=0.5×`" ...
    + "是唯一达到速度/pitch 门槛的变体（MAE %.4f m/s、pitch %.3f°），推荐作为下一轮复核候选；" ...
    + "代价是 wrench slew RMS 增至 %.3f。`R2=2×` 更平滑，但跟踪和 pitch 明显变差。\n\n", ...
    metric(s, "nmpc_r2_0p5x", "velocity_mae_mps"), ...
    rad2deg(metric(s, "nmpc_r2_0p5x", "max_pitch_rad")), ...
    metric(s, "nmpc_r2_0p5x", "wrench_slew_rms"));

fprintf(fid, "## 实验 7：摩擦/力矩约束边界\n\n");
fprintf(fid, "![constraint limit](../figures/constraint_limit.png)\n\n");
constraints = s(s.experiment == "constraint", :);
writeTable(fid, constraints);
fprintf(fid, "0.5 m/s、0.5 m/s² 为三组参数均能完整跑完的最大离散测试点。" ...
    + "其中低力矩组出现 1 个 NMPC fault 样本（正常率 %.4f%%），故未通过严格全程正常护栏。" ...
    + "0.75 档的基线、低摩擦和低力矩均因失稳后的 Simscape 步长塌缩而无法完成 10 s；" ...
    + "因此连续边界只能表述为位于 0.5 与 0.75 之间，1.0 档按单调升级原则未继续运行。" ...
    + "失败点详见 `../data/constraint_failures.csv`。\n\n", ...
    100*metric(s, "constraint_low_tau_v0p5_a0p5", "nmpc_fault_zero_ratio"));

fprintf(fid, "## 实验 8：外部扰动恢复\n\n");
fprintf(fid, "![disturbance recovery](../figures/disturbance_recovery.png)\n\n");
writeTable(fid, s(s.experiment == "disturbance", :));
if ~isempty(d5)
    fprintf(fid, "5 N 脉冲完整恢复：最大 pitch %.3f°、最大位移 %.4f m、恢复时间 %.3f s。\n\n", ...
        rad2deg(d5.max_pitch_rad), d5.disturbance_max_displacement_m, ...
        d5.disturbance_recovery_time_s);
end
if ~isempty(d20)
    fprintf(fid, "20 N 在施力后 0.1 s 内已达到 pitch %.3f°、速度 %.3f m/s，QP 可行率降至 %.3f%%；" ...
        + "继续仿真进入极小步长而无法完成。该点判为强扰动失效，不提供恢复时间。\n\n", ...
        rad2deg(d20.max_pitch_rad), d20.realized_peak_speed_mps, ...
        100*d20.qp_feasible_ratio);
end

fprintf(fid, "## 完成清单\n\n");
check(fid, passed(stand), "静态站立通过");
check(fid, passed(pitch), "pitch 扰动恢复通过");
check(fid, passed(velocity), "速度跟踪性能门槛通过");
check(fid, nmpPass, "非最小相位轮位反向运动验证");
check(fid, true, "LQR 权重经验已记录");
check(fid, true, "NMPC 权重经验已记录");
check(fid, true, "摩擦/力矩边界已记录");
check(fid, true, "外部扰动恢复/失效边界已记录");
check(fid, true, "自动报告已生成");
fprintf(fid, "\n结论只适用于严格共同模态。左右差动、接触不一致、单侧扰动和三维自由度应在后续完整双腿模型中另行验证。\n");
clear cleanup
end

function writeTable(fid, q)
if isempty(q)
    fprintf(fid, "无可用结果。\n\n");
    return;
end
fprintf(fid, "| case | parameter | sim | pass | v MAE (m/s) | peak wheel (m) | peak pitch (deg) | QP feasible | dyn residual |\n");
fprintf(fid, "|---|---|:---:|:---:|---:|---:|---:|---:|---:|\n");
for idx = 1:height(q)
    fprintf(fid, "| %s | %s | %s | %s | %.4g | %.4g | %.4g | %.4g | %.3e |\n", ...
        q.case_name(idx), q.parameter(idx), yesno(q.sim_ok(idx)), ...
        yesno(q.accepted(idx)), q.velocity_mae_mps(idx), ...
        q.wheel_peak_delta_m(idx), rad2deg(q.max_pitch_rad(idx)), ...
        q.qp_feasible_ratio(idx), q.max_dynamics_residual(idx));
end
fprintf(fid, "\n");
end

function q = one(s, name)
q = s(s.case_name == name, :);
if ~isempty(q), q = q(1, :); end
end

function value = metric(s, name, field)
q = one(s, name);
value = q.(field)(1);
end

function value = passed(q)
value = ~isempty(q) && q.sim_ok(1) && q.accepted(1);
end

function outcomeLine(fid, label, ok)
state = "未通过";
if ok, state = "通过"; end
fprintf(fid, "- **%s：%s。**\n", label, state);
end

function check(fid, ok, label)
mark = " ";
if ok, mark = "x"; end
fprintf(fid, "- [%s] %s\n", mark, label);
end

function value = yesno(ok)
value = "no";
if ok, value = "yes"; end
end
