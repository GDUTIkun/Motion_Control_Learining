function reportFile = analyze_common_mode_validation(studyDir)
%ANALYZE_COMMON_MODE_VALIDATION Plot saved cases and write the Markdown report.

if nargin < 1 || isempty(studyDir)
    studyDir = fileparts(mfilename("fullpath"));
end
dataDir = fullfile(studyDir, "data");
figureDir = fullfile(studyDir, "figures");
reportDir = fullfile(studyDir, "report");
ensureFolder(figureDir);
ensureFolder(reportDir);
loaded = load(fullfile(dataDir, "summary.mat"), "summary", "baseline");
summary = loaded.summary;
baseline = loaded.baseline;

plotStand(readCase(dataDir, "stand"), figureDir);
plotPitch(readCase(dataDir, "pitch_2deg"), figureDir);
plotVelocity(readCase(dataDir, "velocity_baseline"), figureDir, baseline.baseNmpc.model.xiEq);
plotSensitivity(summary, figureDir);
plotConstraints(summary, figureDir);
plotDisturbance(readCase(dataDir, "disturbance_20N"), ...
    readCase(dataDir, "disturbance_5N"), figureDir);

reportFile = fullfile(reportDir, "common_mode_validation_report.md");
write_common_mode_validation_report(reportFile, summary, baseline);
fprintf("Common-mode validation report: %s\n", reportFile);
end

function plotStand(d, figureDir)
if isempty(d), return; end
c = colors();
f = newFigure();
tiledlayout(3, 1, "TileSpacing", "compact");
nexttile; plot(d.time_s, d.x_B_m, "Color", c.blue); ylabel("x_B (m)"); grid on;
title("Static stand: base state", "FontWeight", "normal");
nexttile; plot(d.time_s, d.z_B_m - d.z_B_m(1), "Color", c.gold);
ylabel("z_B-z_B(0) (m)"); grid on;
nexttile; plot(d.time_s, rad2deg(d.theta_B_rad), "Color", c.orange);
ylabel("pitch (deg)"); xlabel("Time (s)"); grid on;
saveFigure(f, fullfile(figureDir, "base_state.png"));

f = newFigure();
tiledlayout(3, 1, "TileSpacing", "compact");
labels = ["F_x (N)", "F_z (N)", "M_y (N m)"];
for idx = 1:3
    nexttile;
    plot(d.time_s, d{:, 19+idx}, "Color", c.blue, "DisplayName", "NMPC command"); hold on;
    plot(d.time_s, d{:, 34+idx}, "--", "Color", c.orange, "DisplayName", "QP achieved");
    ylabel(labels(idx)); grid on;
    if idx == 1, title("Static stand: commanded and achieved body wrench", "FontWeight", "normal"); legend("Location", "best"); end
end
xlabel("Time (s)");
saveFigure(f, fullfile(figureDir, "wrench.png"));

residual = d{:, 45:53};
f = newFigure();
tiledlayout(2, 1, "TileSpacing", "compact");
nexttile; semilogy(d.time_s, max(abs(residual), [], 2) + eps, "Color", c.blue);
ylabel("||r_{dyn}||_{inf}"); grid on;
title("Static stand: QP residual and feasibility", "FontWeight", "normal");
nexttile; yyaxis left; plot(d.time_s, d.qp_slack_norm, "Color", c.orange);
ylabel("scaled slack norm"); yyaxis right; stairs(d.time_s, d.qp_feasible, "Color", c.olive);
ylabel("QP feasible"); ylim([-0.05 1.05]); xlabel("Time (s)"); grid on;
saveFigure(f, fullfile(figureDir, "qp_residual.png"));
end

function plotPitch(d, figureDir)
if isempty(d), return; end
c = colors();
f = newFigure();
tiledlayout(3, 1, "TileSpacing", "compact");
nexttile; plot(d.time_s, rad2deg(d.theta_B_rad), "Color", c.blue);
yline(0.1, ":", "Color", c.neutral); yline(-0.1, ":", "Color", c.neutral);
ylabel("pitch (deg)"); grid on; title("2 deg initial-pitch recovery", "FontWeight", "normal");
nexttile; plot(d.time_s, rad2deg(d.dtheta_B_radps), "Color", c.orange);
ylabel("pitch rate (deg/s)"); grid on;
nexttile; plot(d.time_s, d.nmpc_My_Nm, "Color", c.blue, "DisplayName", "NMPC M_y"); hold on;
plot(d.time_s, d.qp_My_Nm, "--", "Color", c.orange, "DisplayName", "QP achieved");
ylabel("M_y (N m)"); xlabel("Time (s)"); legend("Location", "best"); grid on;
saveFigure(f, fullfile(figureDir, "pitch_recovery.png"));
end

function plotVelocity(d, figureDir, xi0)
if isempty(d), return; end
c = colors();
f = newFigure();
tiledlayout(2, 1, "TileSpacing", "compact");
nexttile; plot(d.time_s, d.dx_ref_mps, "--", "Color", c.neutral, "DisplayName", "reference"); hold on;
plot(d.time_s, d.dx_B_mps, "Color", c.blue, "DisplayName", "actual");
ylabel("velocity (m/s)"); legend("Location", "best"); grid on;
title("Velocity round trip", "FontWeight", "normal");
nexttile; plot(d.time_s, d.x_ref_m, "--", "Color", c.neutral, "DisplayName", "reference"); hold on;
plot(d.time_s, d.x_B_m, "Color", c.blue, "DisplayName", "actual");
ylabel("x_B (m)"); xlabel("Time (s)"); legend("Location", "best"); grid on;
saveFigure(f, fullfile(figureDir, "velocity_tracking.png"));

f = newFigure();
plot(d.time_s, d.xi_ideal_abs_m - xi0, ":", "LineWidth", 1.5, ...
    "Color", c.neutral, "DisplayName", "WIPM ideal"); hold on;
plot(d.time_s, d.xi_raw_m - xi0, "--", "Color", c.gold, "DisplayName", "LQR raw");
plot(d.time_s, d.xi_ref_m - xi0, "Color", c.orange, "DisplayName", "governed reference");
plot(d.time_s, d.xi_m - xi0, "Color", c.blue, "DisplayName", "actual");
yline(0, "Color", c.neutral, "HandleVisibility", "off"); xlabel("Time (s)"); ylabel("xi-xi_0 (m)");
title("Common wheel-position planning and tracking", "FontWeight", "normal");
legend("Location", "best"); grid on;
saveFigure(f, fullfile(figureDir, "wheel_position.png"));

f = newFigure();
tiledlayout(3, 1, "TileSpacing", "compact");
nexttile; plot(d.time_s, d.ax_ref_mps2, "Color", c.blue); yline(0, "Color", c.neutral);
ylabel("a_{ref} (m/s^2)"); title("Non-minimum-phase wheel pre-action", "FontWeight", "normal"); grid on;
nexttile; plot(d.time_s, d.xi_ideal_delta_m, ":", "LineWidth", 1.5, "Color", c.neutral, "DisplayName", "ideal"); hold on;
plot(d.time_s, d.xi_ref_m-xi0, "Color", c.orange, "DisplayName", "reference");
plot(d.time_s, d.xi_m-xi0, "Color", c.blue, "DisplayName", "actual");
yline(0, "Color", c.neutral, "HandleVisibility", "off"); ylabel("xi-xi_0 (m)"); legend("Location", "best"); grid on;
nexttile; plot(d.time_s, d.xi_m-d.xi_ref_m, "Color", c.blue);
ylabel("tracking error (m)"); xlabel("Time (s)"); grid on;
saveFigure(f, fullfile(figureDir, "non_minimum_phase.png"));
end

function plotSensitivity(s, figureDir)
c = colors();
base = s(s.case_name == "velocity_baseline", :);
lqr = [base; s(s.experiment == "lqr_sensitivity", :)];
if ~isempty(lqr)
    ratios = [1; parseRatio(lqr.parameter(2:end))];
    [ratios, order] = sort(ratios); lqr = lqr(order, :);
    f = newFigure();
    tiledlayout(1, 3, "TileSpacing", "compact");
    plotBars(ratios, lqr.velocity_mae_mps, "Velocity MAE (m/s)", c.blue);
    plotBars(ratios, lqr.wheel_peak_delta_m, "Peak |xi-xi_0| (m)", c.orange);
    plotBars(ratios, rad2deg(lqr.max_pitch_rad), "Peak pitch (deg)", c.gold);
    sgtitle("WIPM-LQR Q/R sensitivity", "FontWeight", "normal");
    saveFigure(f, fullfile(figureDir, "lqr_sensitivity.png"));
end
nmpc = [base; s(s.experiment == "nmpc_sensitivity", :)];
if ~isempty(nmpc)
    f = newFigure();
    tiledlayout(2, 2, "TileSpacing", "compact");
    labels = ["baseline"; nmpc.parameter(2:end)];
    plotCategoryBars(labels, nmpc.wrench_slew_rms, "Wrench slew RMS", c.blue);
    plotCategoryBars(labels, rad2deg(nmpc.max_pitch_rad), "Peak pitch (deg)", c.orange);
    plotCategoryBars(labels, nmpc.max_slack_norm, "Max slack norm", c.gold);
    plotCategoryBars(labels, 1e3*nmpc.nmpc_cpu_max_s, "Max NMPC CPU (ms)", c.olive);
    sgtitle("NMPC weight sensitivity", "FontWeight", "normal");
    saveFigure(f, fullfile(figureDir, "nmpc_sensitivity.png"));
end
end

function plotConstraints(s, figureDir)
q = s(s.experiment == "constraint", :);
if isempty(q), return; end
labels = q.parameter;
c = colors();
f = newFigure();
tiledlayout(2, 1, "TileSpacing", "compact");
nexttile; hold on;
groups = unique(labels, "stable");
palette = [c.blue; c.orange; c.gold; c.olive];
markers = ["o", "s", "d", "^"];
for idx = 1:numel(groups)
    selected = labels == groups(idx);
    scatter(q.command_a_mps2(selected), q.realized_peak_accel_mps2(selected), ...
        45, palette(idx, :), markers(idx), "filled", "DisplayName", groups(idx));
end
plot([0, max(q.command_a_mps2)], [0, max(q.command_a_mps2)], ":", ...
    "Color", c.neutral, "HandleVisibility", "off");
failed = ~q.sim_ok;
groupOffset = zeros(height(q), 1);
for idx = 1:numel(groups)
    groupOffset(labels == groups(idx)) = 0.012*(idx-2);
end
scatter(q.command_a_mps2(failed) + groupOffset(failed), ...
    0.02*ones(nnz(failed), 1), 60, "x", "MarkerEdgeColor", c.neutral, ...
    "LineWidth", 1.5, "HandleVisibility", "off");
text(0.75, 0.06, "simulation failed", "HorizontalAlignment", "center");
text(1.0, 0.06, "skipped", "HorizontalAlignment", "center");
xlabel("command acceleration (m/s^2)"); ylabel("realized peak (m/s^2)"); grid on;
title("Constraint boundary cases", "FontWeight", "normal"); legend("Location", "best");
nexttile; hold on;
for idx = 1:numel(groups)
    selected = labels == groups(idx);
    scatter(q.command_v_mps(selected), q.qp_feasible_ratio(selected), ...
        45, palette(idx, :), markers(idx), "filled", "DisplayName", groups(idx));
end
scatter(q.command_v_mps(failed) + groupOffset(failed), ...
    0.02*ones(nnz(failed), 1), 60, "x", "MarkerEdgeColor", c.neutral, ...
    "LineWidth", 1.5, "HandleVisibility", "off");
xlabel("command velocity (m/s)"); ylabel("QP feasible ratio"); ylim([0 1.02]); grid on;
xlim([0.45 1.05]);
saveFigure(f, fullfile(figureDir, "constraint_limit.png"));
end

function plotDisturbance(d20, d5, figureDir)
if isempty(d20) && isempty(d5), return; end
c = colors();
f = newFigure();
tiledlayout(3, 1, "TileSpacing", "compact");
nexttile; hold on;
if ~isempty(d5), plot(d5.time_s, 5*(d5.time_s >= 2.5 & d5.time_s < 3.0), ...
        "Color", c.blue, "DisplayName", "5 N full run"); end
if ~isempty(d20), plot(d20.time_s, 20*(d20.time_s >= 2.5 & d20.time_s < 3.0), ...
        "Color", c.orange, "DisplayName", "20 N early window"); end
ylabel("F_{ext} (N)"); title("Horizontal disturbance response", "FontWeight", "normal");
legend("Location", "best"); grid on;
nexttile; hold on;
if ~isempty(d5), plot(d5.time_s, rad2deg(d5.theta_B_rad), "Color", c.blue, "DisplayName", "5 N"); end
if ~isempty(d20), plot(d20.time_s, rad2deg(d20.theta_B_rad), "Color", c.orange, "DisplayName", "20 N"); end
ylabel("pitch (deg)"); legend("Location", "best"); grid on;
nexttile; hold on;
if ~isempty(d5), plot(d5.time_s, d5.x_B_m-d5.x_B_m(1), "Color", c.blue, "DisplayName", "5 N"); end
if ~isempty(d20), plot(d20.time_s, d20.x_B_m-d20.x_B_m(1), "Color", c.orange, "DisplayName", "20 N"); end
ylabel("Delta x_B (m)"); xlabel("Time (s)"); legend("Location", "best"); grid on;
saveFigure(f, fullfile(figureDir, "disturbance_recovery.png"));
end

function writeReport(path, s, b)
fid = fopen(path, "w", "n", "UTF-8");
assert(fid >= 0, "Cannot write report: %s", path);
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, "# 严格共同模态轮腿控制算法验证报告\n\n");
fprintf(fid, "> 生成时间：%s（Asia/Shanghai）  \n", string(datetime("now", "TimeZone", "Asia/Shanghai")));
fprintf(fid, "> 模型：`source_common.slx`；数据源：本目录 `../data/*.csv`。\n\n");
fprintf(fid, "## 技术摘要\n\n");
writeOutcome(fid, "静态站立", outcome(s, "stand"));
writeOutcome(fid, "2° pitch 恢复", outcome(s, "pitch"));
writeOutcome(fid, "速度跟踪", rowOutcome(s, "velocity_baseline"));
velocity = row(s, "velocity_baseline");
if ~isempty(velocity)
    nmpPass = velocity.nmp_ideal_direction_ratio > 0.99 ...
        && velocity.nmp_ref_direction_ratio > 0.5 ...
        && velocity.nmp_actual_direction_ratio > 0.5;
    writeOutcome(fid, "非最小相位反向预动作", nmpPass);
end
fprintf(fid, "\n本报告只验证平面严格共同模态，不外推到左右差动或三维运动。扫描结果只记录经验，不自动改写控制器参数。\n\n");

fprintf(fid, "## 模型与控制链路\n\n");
fprintf(fid, "验证自由度为基座 $[x_B,z_B,\\theta_B]$ 与一组左右完全同步的共同腿变量。执行链为：\n\n");
fprintf(fid, "`WIPM-LQR wheel planning -> 8-state NMPC body wrench -> strict common-mode WBC-QP -> summed joint torque -> Simscape plant`。\n\n");
fprintf(fid, "NMPC 周期 %.3f s、节点数 %d、预测时域 %.2f s；静态轮位 $\\xi_0=%.7f$ m。\n\n", ...
    b.baseNmpc.Ts, b.baseNmpc.N, b.baseNmpc.Ts*b.baseNmpc.N, b.baseNmpc.model.xiEq);

fprintf(fid, "## 指标与护栏定义\n\n");
fprintf(fid, "- 轮位幅值统一使用 $\\Delta\\xi=\\xi-\\xi_0$。理论前馈为 $\\Delta\\xi_{ideal}=-H a_x^{ref}/g$。\n");
fprintf(fid, "- 有效数据要求 NMPC status/fault 全程正常、QP 可行率 >99%%、动力学残差无穷范数 <1e-6。\n");
fprintf(fid, "- 接触总竖向力按整机质量校核；QP body wrench 的 $F_z$ 按 NMPC 降阶上体质量校核。\n");
fprintf(fid, "- pitch 整定带为 max(0.1°, 初值的 2%%)，速度 MAE 相对轨迹参考计算。\n\n");

fprintf(fid, "## 实验 1：静态站立\n\n![base state](../figures/base_state.png)\n\n");
fprintf(fid, "![wrench](../figures/wrench.png)\n\n![QP residual](../figures/qp_residual.png)\n\n");
writeRows(fid, s(s.experiment == "stand", :));
fprintf(fid, "## 实验 2：初始 pitch 扰动恢复\n\n![pitch recovery](../figures/pitch_recovery.png)\n\n");
writeRows(fid, s(s.experiment == "pitch", :));
fprintf(fid, "## 实验 3：速度跟踪\n\n![velocity tracking](../figures/velocity_tracking.png)\n\n");
fprintf(fid, "![wheel position](../figures/wheel_position.png)\n\n");
writeRows(fid, s(s.case_name == "velocity_baseline", :));
fprintf(fid, "## 实验 4：非最小相位轮位规划\n\n![non-minimum phase](../figures/non_minimum_phase.png)\n\n");
if ~isempty(velocity)
    fprintf(fid, "正加速区间方向正确率：ideal %.1f%%、LQR/governor reference %.1f%%、actual %.1f%%；实际/理论峰值比 %.3f。\n\n", ...
        100*velocity.nmp_ideal_direction_ratio, 100*velocity.nmp_ref_direction_ratio, ...
        100*velocity.nmp_actual_direction_ratio, velocity.nmp_actual_to_ideal_ratio);
end
fprintf(fid, "## 实验 5：WIPM-LQR Q/R 敏感性\n\n![LQR sensitivity](../figures/lqr_sensitivity.png)\n\n");
writeRows(fid, [s(s.case_name == "velocity_baseline", :); s(s.experiment == "lqr_sensitivity", :)]);
fprintf(fid, "共同缩放 Q 与 R 不改变 LQR 增益，因此扫描采用等价的 Q/R 比例 0.5×、1×、2×。\n\n");
fprintf(fid, "## 实验 6：NMPC 权重敏感性\n\n![NMPC sensitivity](../figures/nmpc_sensitivity.png)\n\n");
writeRows(fid, [s(s.case_name == "velocity_baseline", :); s(s.experiment == "nmpc_sensitivity", :)]);
fprintf(fid, "`R1` 是对 wrench 参考偏差的惩罚，`R2` 是输入增量惩罚；两者被编译进 acados 求解器，每个变体使用独立生成目录。推荐值只在所有护栏通过的已测组合中选择，不做外推。\n\n");
fprintf(fid, "## 实验 7：约束边界\n\n![constraint limit](../figures/constraint_limit.png)\n\n");
writeRows(fid, [s(s.case_name == "velocity_baseline", :); s(s.experiment == "constraint", :)]);
fprintf(fid, "`low_mu` 只改变下层共同模态 QP 摩擦锥，不改变 Simscape 轮地材料参数，也不改变已编译 NMPC 的 drive coefficient。边界结论仅限离散测试网格，不外推连续极限。\n\n");
fprintf(fid, "## 实验 8：20 N 水平外扰恢复\n\n![disturbance recovery](../figures/disturbance_recovery.png)\n\n");
writeRows(fid, s(s.experiment == "disturbance", :));

fprintf(fid, "## 经验总结与完成清单\n\n");
labels = ["静态站立通过"; "pitch 扰动恢复通过"; "速度跟踪通过"; ...
    "非最小相位反向运动验证"; "LQR 权重经验已记录"; ...
    "NMPC 权重经验已记录"; "摩擦/力矩边界已记录"; ...
    "外部扰动恢复已测试"; "自动报告已生成"];
checks = [outcome(s, "stand"); outcome(s, "pitch"); ...
    rowOutcome(s, "velocity_baseline"); ...
    ~isempty(velocity) && velocity.nmp_actual_direction_ratio > 0.5; ...
    allSimulated(s, "lqr_sensitivity"); allSimulated(s, "nmpc_sensitivity"); ...
    allSimulated(s, "constraint"); allSimulated(s, "disturbance"); true];
for idx = 1:numel(labels)
    mark = " "; if checks(idx), mark = "x"; end
    fprintf(fid, "- [%s] %s\n", mark, labels(idx));
end
fprintf(fid, "\n未通过项保留为实验结论，不触发自动调参。后续完整双腿模型应另行验证左右差动、接触不一致、单侧扰动和三维自由度。\n");
clear cleanup
end

function writeRows(fid, q)
if isempty(q)
    fprintf(fid, "无可用结果。\n\n"); return;
end
fprintf(fid, "| case | parameter | sim | pass | v MAE (m/s) | peak wheel (m) | peak pitch (deg) | QP feasible | dyn residual | max torque (N m) |\n");
fprintf(fid, "|---|---|:---:|:---:|---:|---:|---:|---:|---:|---:|\n");
for idx = 1:height(q)
    fprintf(fid, "| %s | %s | %s | %s | %.4g | %.4g | %.4g | %.4g | %.3e | %.4g |\n", ...
        q.case_name(idx), q.parameter(idx), tf(q.sim_ok(idx)), tf(q.accepted(idx)), ...
        q.velocity_mae_mps(idx), q.wheel_peak_delta_m(idx), ...
        rad2deg(q.max_pitch_rad(idx)), q.qp_feasible_ratio(idx), ...
        q.max_dynamics_residual(idx), q.max_torque_Nm(idx));
end
fprintf(fid, "\n");
end

function value = outcome(s, experiment)
q = s(s.experiment == experiment, :);
value = ~isempty(q) && all(q.sim_ok) && all(q.accepted);
end

function value = rowOutcome(s, name)
q = s(s.case_name == name, :);
value = ~isempty(q) && q.sim_ok(1) && q.accepted(1);
end

function value = allSimulated(s, experiment)
q = s(s.experiment == experiment, :);
value = ~isempty(q) && all(q.sim_ok);
end

function writeOutcome(fid, label, passed)
state = "未通过"; if passed, state = "通过"; end
fprintf(fid, "- **%s：%s。**\n", label, state);
end

function q = row(s, name)
q = s(s.case_name == name, :);
if ~isempty(q), q = q(1, :); end
end

function result = tf(value)
result = "no"; if value, result = "yes"; end
end

function d = readCase(dataDir, name)
file = fullfile(dataDir, name + ".csv");
if isfile(file), d = readtable(file); else, d = table(); end
end

function values = parseRatio(labels)
values = NaN(numel(labels), 1);
for idx = 1:numel(labels)
    token = regexp(labels(idx), "([0-9.]+)x", "tokens", "once");
    if ~isempty(token), values(idx) = str2double(token{1}); end
end
end

function plotBars(x, y, label, color)
nexttile; bar(x, y, 0.55, "FaceColor", color); xlabel("Q/R ratio scale"); ylabel(label); grid on;
end

function plotCategoryBars(labels, y, titleText, color)
nexttile; bar(categorical(labels, labels), y, 0.55, "FaceColor", color);
ylabel(titleText); grid on; xtickangle(20);
end

function f = newFigure()
f = figure("Visible", "off", "Color", "white", "Position", [100 100 1100 720]);
set(f, "DefaultAxesFontName", "Microsoft YaHei", "DefaultAxesFontSize", 10);
end

function saveFigure(f, path)
exportgraphics(f, path, "Resolution", 180);
close(f);
end

function c = colors()
c = struct("blue", [0.12 0.35 0.62], "gold", [0.76 0.55 0.12], ...
    "orange", [0.85 0.35 0.12], "olive", [0.42 0.49 0.16], ...
    "neutral", [0.35 0.37 0.40]);
end

function ensureFolder(path)
if ~isfolder(path), mkdir(path); end
end
