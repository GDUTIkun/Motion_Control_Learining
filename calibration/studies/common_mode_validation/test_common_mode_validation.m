function summary = test_common_mode_validation(scope)
%TEST_COMMON_MODE_VALIDATION Run the strict common-mode validation campaign.

if nargin < 1 || isempty(scope)
    scope = "all";
end
scope = lower(string(scope));
validScopes = ["all", "smoke", "nmpc_r2_2x", "remaining"];
isSingleConstraint = startsWith(scope, "constraint_");
isSingleDisturbance = any(scope == ["disturbance_20n", "disturbance_5n"]);
assert(any(scope == validScopes) || isSingleConstraint || isSingleDisturbance, ...
    "Unknown validation scope: %s", scope);

studyDir = fileparts(mfilename("fullpath"));
repoRoot = fileparts(fileparts(fileparts(studyDir)));
modelDir = fullfile(repoRoot, "model", "simulate", "two_legs");
dataDir = fullfile(studyDir, "data");
ensureFolder(dataDir);
addpath(studyDir, modelDir);

oldDir = cd(modelDir);
evalin("base", "run('" + replace(fullfile(modelDir, "startup.m"), ...
    "'", "''") + "')");
load_system("source_common");
model = "source_common";
qpBlock = model + "/PD_only/Coupled QP";
splitBlock = model + "/PD_only/Coupled QP Split";
initFcn = get_param(model, "InitFcn");
qpFcn = get_param(qpBlock, "MATLABFcn");
qpWidth = get_param(qpBlock, "OutputDimensions");
splitOutputs = get_param(splitBlock, "Outputs");
pulseBlocks = find_system(model, "LookUnderMasks", "all", ...
    "FollowLinks", "on", "BlockType", "DiscretePulseGenerator");
cleanup = onCleanup(@() restoreModel(oldDir, model, initFcn, qpBlock, ...
    qpFcn, qpWidth, splitBlock, splitOutputs));
set_param(model, "InitFcn", "");
set_param(qpBlock, "MATLABFcn", "common_mode_qp_validation_signal", ...
    "OutputDimensions", "28");
set_param(splitBlock, "Outputs", "[3 3 3 3 1 1 2 2 1 9]");
set_param(model, "SimulationCommand", "update");

baseline = captureBaseline(modelDir);
rows = repmat(emptyMetrics(), 0, 1);
v = caseDef("velocity_baseline", "velocity", "baseline");
v.mode = "velocity";

if scope == "nmpc_r2_2x"
    variant = buildNmpcVariant("r2_2x", baseline, modelDir);
    c = caseDef("nmpc_r2_2x", "nmpc_sensitivity", "r2_2x");
    c.mode = "velocity";
    c.baseNmpc = variant;
    rows(end+1) = safeRun(c, baseline, dataDir, pulseBlocks); %#ok<AGROW>
    summary = mergeExistingMetrics(rows, dataDir);
    writetable(summary, fullfile(dataDir, "summary.csv"));
    save(fullfile(dataDir, "summary.mat"), "summary", "baseline");
    analyze_common_mode_validation(studyDir);
    clear cleanup
    return;
end

if isSingleConstraint
    c = v;
    c.name = scope;
    c.experiment = "constraint";
    if contains(scope, "v0p75_a0p75")
        c.commandV = 0.75;
        c.commandA = 0.75;
    elseif contains(scope, "v1_a1")
        c.commandV = 1.0;
        c.commandA = 1.0;
    else
        c.commandV = 0.5;
        c.commandA = 0.5;
    end
    if contains(scope, "low_mu")
        c.parameter = "QP mu=0.20";
        c.mu = 0.20;
    elseif contains(scope, "low_tau")
        c.parameter = "tauMax=0.25x";
        c.tauScale = 0.25;
    else
        c.parameter = "baseline";
    end
    rows(end+1) = safeRun(c, baseline, dataDir, pulseBlocks); %#ok<AGROW>
    summary = mergeExistingMetrics(rows, dataDir);
    writetable(summary, fullfile(dataDir, "summary.csv"));
    save(fullfile(dataDir, "summary.mat"), "summary", "baseline");
    analyze_common_mode_validation(studyDir);
    clear cleanup
    return;
end

if isSingleDisturbance
    if scope == "disturbance_20n"
        d = caseDef("disturbance_20N", "disturbance", ...
            "20 N, 2.5-3.0 s; early-response window");
        d.pulseAmplitude = 20;
        d.stopTime = 2.6;
    else
        d = caseDef("disturbance_5N", "disturbance", "5 N, 2.5-3.0 s");
        d.pulseAmplitude = 5;
    end
    rows(end+1) = safeRun(d, baseline, dataDir, pulseBlocks); %#ok<AGROW>
    summary = mergeExistingMetrics(rows, dataDir);
    writetable(summary, fullfile(dataDir, "summary.csv"));
    save(fullfile(dataDir, "summary.mat"), "summary", "baseline");
    analyze_common_mode_validation(studyDir);
    clear cleanup
    return;
end

if scope ~= "remaining"
rows(end+1) = safeRun(caseDef("stand", "stand", "baseline"), baseline, dataDir, pulseBlocks); %#ok<AGROW>
if scope == "smoke"
    summary = struct2table(rows);
    writetable(summary, fullfile(dataDir, "summary.csv"));
    save(fullfile(dataDir, "summary.mat"), "summary", "baseline");
    analyze_common_mode_validation(studyDir);
    clear cleanup
    return;
end
p = caseDef("pitch_2deg", "pitch", "2 deg initial pitch");
p.x0(3) = deg2rad(2);
rows(end+1) = safeRun(p, baseline, dataDir, pulseBlocks); %#ok<AGROW>
rows(end+1) = safeRun(v, baseline, dataDir, pulseBlocks); %#ok<AGROW>

for ratio = [0.5, 2]
    c = v;
    c.name = "lqr_qr_" + tag(ratio) + "x";
    c.experiment = "lqr_sensitivity";
    c.parameter = "Q/R ratio " + ratio + "x";
    c.lqrRatio = ratio;
    rows(end+1) = safeRun(c, baseline, dataDir, pulseBlocks); %#ok<AGROW>
end

nmpcCases = ["r1_0p5x", "r1_2x", "r2_0p5x", "r2_2x"];
for name = nmpcCases
    try
        variant = buildNmpcVariant(name, baseline, modelDir);
        c = v;
        c.name = "nmpc_" + name;
        c.experiment = "nmpc_sensitivity";
        c.parameter = name;
        c.baseNmpc = variant;
        rows(end+1) = safeRun(c, baseline, dataDir, pulseBlocks); %#ok<AGROW>
    catch exception
        failed = emptyMetrics();
        failed.case_name = "nmpc_" + name;
        failed.experiment = "nmpc_sensitivity";
        failed.parameter = name;
        failed.error_message = string(exception.message);
        rows(end+1) = failed; %#ok<AGROW>
    end
end
end

profiles = [0.5, 0.5; 0.75, 0.75; 1.0, 1.0];
for idx = 1:size(profiles, 1)
    if idx > 1
        c = v;
        c.name = "constraint_baseline_" + demandTag(profiles(idx, :));
        c.experiment = "constraint";
        c.parameter = "baseline";
        c.commandV = profiles(idx, 1);
        c.commandA = profiles(idx, 2);
        rows(end+1) = safeRun(c, baseline, dataDir, pulseBlocks); %#ok<AGROW>
    end
    c = v;
    c.name = "constraint_low_mu_" + demandTag(profiles(idx, :));
    c.experiment = "constraint";
    c.parameter = "QP mu=0.20";
    c.commandV = profiles(idx, 1);
    c.commandA = profiles(idx, 2);
    c.mu = 0.20;
    rows(end+1) = safeRun(c, baseline, dataDir, pulseBlocks); %#ok<AGROW>

    c = v;
    c.name = "constraint_low_tau_" + demandTag(profiles(idx, :));
    c.experiment = "constraint";
    c.parameter = "tauMax=0.25x";
    c.commandV = profiles(idx, 1);
    c.commandA = profiles(idx, 2);
    c.tauScale = 0.25;
    rows(end+1) = safeRun(c, baseline, dataDir, pulseBlocks); %#ok<AGROW>
end

d = caseDef("disturbance_20N", "disturbance", "20 N, 2.5-3.0 s");
d.pulseAmplitude = 20;
rows(end+1) = safeRun(d, baseline, dataDir, pulseBlocks); %#ok<AGROW>

if scope == "remaining"
    summary = mergeExistingMetrics(rows, dataDir);
else
    summary = struct2table(rows);
end
writetable(summary, fullfile(dataDir, "summary.csv"));
save(fullfile(dataDir, "summary.mat"), "summary", "baseline");
analyze_common_mode_validation(studyDir);
clear cleanup
end

function summary = mergeExistingMetrics(rows, dataDir)
allRows = repmat(emptyMetrics(), 0, 1);
files = dir(fullfile(dataDir, "*.mat"));
for idx = 1:numel(files)
    if strcmp(files(idx).name, "summary.mat")
        continue;
    end
    stored = load(fullfile(files(idx).folder, files(idx).name), "metrics");
    if isfield(stored, "metrics")
        allRows(end+1) = stored.metrics; %#ok<AGROW>
    end
end
for idx = 1:numel(rows)
    match = find([allRows.case_name] == rows(idx).case_name, 1);
    if isempty(match)
        allRows(end+1) = rows(idx); %#ok<AGROW>
    else
        allRows(match) = rows(idx);
    end
end
summary = struct2table(allRows);
summary = sortrows(summary, ["experiment", "case_name"]);
end

function baseline = captureBaseline(modelDir)
baseline.modelDir = modelDir;
baseline.base = evalin("base", "base");
baseline.leg = evalin("base", "leg");
baseline.ctrl = evalin("base", "ctrl");
baseline.traj = evalin("base", "traj");
baseline.baseLqr = evalin("base", "baseLqr");
baseline.wheelLqr = evalin("base", "wheelLqr");
baseline.baseNmpc = evalin("base", "baseNmpc");
end

function c = caseDef(name, experiment, parameter)
c = struct("name", string(name), "experiment", string(experiment), ...
    "parameter", string(parameter), "mode", "stand", ...
    "stopTime", 10, "x0", zeros(6, 1), "lqrRatio", 1, ...
    "baseNmpc", [], "mu", NaN, "tauScale", 1, ...
    "commandV", 0.5, "commandA", 0.5, "pulseAmplitude", 0);
end

function result = safeRun(c, baseline, dataDir, pulseBlocks)
try
    [samples, metadata, metrics] = runCase(c, baseline, pulseBlocks);
    csvFile = fullfile(dataDir, c.name + ".csv");
    matFile = fullfile(dataDir, c.name + ".mat");
    writetable(samples, csvFile);
    save(matFile, "samples", "metadata", "metrics", "-v7.3");
    result = metrics;
catch exception
    result = emptyMetrics();
    result.case_name = c.name;
    result.experiment = c.experiment;
    result.parameter = c.parameter;
    result.command_v_mps = c.commandV;
    result.command_a_mps2 = c.commandA;
    result.error_message = string(exception.message);
    warning("common_mode_validation:CaseFailed", "%s: %s", ...
        c.name, exception.message);
end
end

function [samples, metadata, metrics] = runCase(c, baseline, pulseBlocks)
assignBaseline(baseline);
baseNmpc = baseline.baseNmpc;
if ~isempty(c.baseNmpc)
    baseNmpc = c.baseNmpc;
end
assignin("base", "baseNmpc", baseNmpc);
configure_base_nmpc_simulink(false, "source_common");
configure_base_tracking_case(c.mode, "lqr", "source_common");

base = evalin("base", "base");
baseLqr = evalin("base", "baseLqr");
traj = evalin("base", "traj");
ctrl = evalin("base", "ctrl");
leg = evalin("base", "leg");
trajectory = base.trajectory;
if c.mode == "velocity"
    trajectory.cruiseVelocity = c.commandV;
    trajectory.accelDuration = abs(c.commandV / c.commandA);
    trajectory.decelDuration = trajectory.accelDuration;
end
base.trajectory = trajectory;
baseLqr.trajectory = trajectory;
traj.wheelLqrR = baseline.traj.wheelLqrR / c.lqrRatio;
wheelLqr = wheel_position_lqr_design(base, leg, traj);
if isfinite(c.mu)
    ctrl.mu = c.mu;
end
ctrl.tauMax = c.tauScale * baseline.ctrl.tauMax;
assignin("base", "base", base);
assignin("base", "baseLqr", baseLqr);
assignin("base", "traj", traj);
assignin("base", "wheelLqr", wheelLqr);
assignin("base", "ctrl", ctrl);
set_initial_base_state(c.x0);

for idx = 1:numel(pulseBlocks)
    set_param(pulseBlocks{idx}, "Amplitude", string(c.pulseAmplitude), ...
        "Period", "10", "PulseWidth", "5", "PhaseDelay", "2.5");
end
clearFunctions();
out = sim("source_common", "StopTime", string(c.stopTime), ...
    "Timeout", 60, "ReturnWorkspaceOutputs", "on");
[samples, metrics] = collectCase(out.logsout, c, baseNmpc, base, leg, ctrl);
metadata = struct("caseDefinition", c, "baseNmpc", baseNmpc, ...
    "wheelLqr", wheelLqr, "ctrl", ctrl, ...
    "generatedAt", datetime("now", "TimeZone", "Asia/Shanghai"), ...
    "model", "source_common", ...
    "lowMuScope", "Lower common-mode QP friction cone only");
end

function assignBaseline(b)
assignin("base", "base", b.base);
assignin("base", "leg", b.leg);
assignin("base", "ctrl", b.ctrl);
assignin("base", "traj", b.traj);
assignin("base", "baseLqr", b.baseLqr);
assignin("base", "wheelLqr", b.wheelLqr);
assignin("base", "baseNmpc", b.baseNmpc);
end

function config = buildNmpcVariant(name, baseline, modelDir)
config = baseline.baseNmpc;
if endsWith(name, "0p5x")
    scale = 0.5;
else
    scale = 2.0;
end
if startsWith(name, "r1")
    config.R1 = scale * baseline.baseNmpc.R1;
else
    config.R2 = scale * baseline.baseNmpc.R2;
end
config.W_e = config.Q;
safeName = replace(name, ".", "p");
config.solverName = "base_wheel_8state_nmpc_val_" + safeName;
config.sfunName = "acados_solver_sfunction_" + config.solverName;
config.buildTag = safeName;
config.generatedDir = fullfile(modelDir, "generated", ...
    "base_wheel_8state_nmpc_validation", safeName);
config.available = isfile(fullfile(config.generatedDir, ...
    config.sfunName + "." + mexext));
config.referenceSize = 14*config.N + 8;
build_base_nmpc_solver(false, false, config);
config.available = true;
end

function [tableData, metrics] = collectCase(logs, c, baseNmpc, base, leg, ctrl)
[t, common] = signalRows(logs, "commonWheelStateSignal");
wheel = atTime(logs, "wheelPositionLqrReference", t, false);
nmpc = atTime(logs, "nmpcBodyWrench", t, false);
status = atTime(logs, "nmpcStatus", t, true);
fault = atTime(logs, "nmpcFault", t, true);
cpu = atTime(logs, "nmpcCpuTime", t, false);
qp = atTime(logs, "coupledQpSignal", t, true);
symmetry = atTime(logs, "symmetryLegState", t, false);
assert(size(common, 2) == 10 && size(wheel, 2) == 4 ...
    && size(nmpc, 2) == 3 && size(qp, 2) == 28 ...
    && size(symmetry, 2) == 12, "Unexpected validation log width.");

reference = zeros(numel(t), 3);
baseLqr = evalin("base", "baseLqr");
for idx = 1:numel(t)
    [xRef, aRef] = floating_base_reference(t(idx), baseLqr);
    reference(idx, :) = [xRef(1), xRef(4), aRef(1)];
end
xi0 = baseNmpc.model.xiEq;
xiIdealDelta = -common(:, 10) ./ base.g .* reference(:, 3);
xiIdealAbs = xi0 + xiIdealDelta;
matrix = [t, common(:, 2:10), reference, xiIdealDelta, xiIdealAbs, ...
    wheel, nmpc, status, fault, cpu, qp, symmetry];
names = sampleNames();
assert(size(matrix, 2) == numel(names));
tableData = array2table(matrix, "VariableNames", cellstr(names));
metrics = computeMetrics(tableData, c, baseNmpc, base, leg, ctrl);
end

function names = sampleNames()
names = ["time_s", "x_B_m", "z_B_m", "theta_B_rad", "dx_B_mps", ...
    "dz_B_mps", "dtheta_B_radps", "xi_m", "dxi_mps", "H_m", ...
    "x_ref_m", "dx_ref_mps", "ax_ref_mps2", "xi_ideal_delta_m", ...
    "xi_ideal_abs_m", "xi_ref_m", "dxi_ref_mps", ...
    "ddxi_ref_mps2", "xi_raw_m", "nmpc_Fx_N", "nmpc_Fz_N", ...
    "nmpc_My_Nm", "nmpc_status", "nmpc_fault", "nmpc_cpu_s", ...
    "tau_L_hip_Nm", "tau_L_knee_Nm", "tau_L_wheel_Nm", ...
    "tau_R_hip_Nm", "tau_R_knee_Nm", "tau_R_wheel_Nm", ...
    "qp_slack_Fx_N", "qp_slack_Fz_N", "qp_slack_My_Nm", ...
    "qp_Fx_N", "qp_Fz_N", "qp_My_Nm", "qp_slack_norm", ...
    "qp_feasible", "contact_L_Fx_N", "contact_L_Fz_N", ...
    "contact_R_Fx_N", "contact_R_Fz_N", "qp_exitflag", ...
    "dynamics_residual_1", "dynamics_residual_2", "dynamics_residual_3", ...
    "dynamics_residual_4", "dynamics_residual_5", "dynamics_residual_6", ...
    "dynamics_residual_7", "dynamics_residual_8", "dynamics_residual_9", ...
    "qL_hip_rad", "qL_knee_rad", "qL_wheel_rad", ...
    "dqL_hip_radps", "dqL_knee_radps", "dqL_wheel_radps", ...
    "qR_hip_rad", "qR_knee_rad", "qR_wheel_rad", ...
    "dqR_hip_radps", "dqR_knee_radps", "dqR_wheel_radps"];
end

function m = computeMetrics(d, c, baseNmpc, base, leg, ctrl)
m = emptyMetrics();
m.case_name = c.name;
m.experiment = c.experiment;
m.parameter = c.parameter;
m.command_v_mps = c.commandV;
m.command_a_mps2 = c.commandA;
m.mu = ctrl.mu;
m.tau_scale = c.tauScale;
m.sim_ok = true;
x = d{:,:};
m.all_finite = all(isfinite(x), "all");
m.nmpc_status_zero_ratio = mean(d.nmpc_status == 0);
m.nmpc_fault_zero_ratio = mean(d.nmpc_fault == 0);
m.qp_feasible_ratio = mean(d.qp_feasible > 0.5);
residual = d{:, 45:53};
m.max_dynamics_residual = max(vecnorm(residual, inf, 2));
m.max_slack_norm = max(d.qp_slack_norm);
tau = d{:, 26:31};
m.max_torque_Nm = max(abs(tau), [], "all");
tauLimit = [ctrl.tauMax(:); ctrl.tauMax(:)].';
m.torque_saturation_ratio = mean(abs(tau) >= tauLimit - 1e-6, "all");
sym = d{:, 54:65};
symDiff = sym(:, [1,2,4,5,6]) - sym(:, [7,8,10,11,12]);
m.max_symmetry_difference = max(abs(symDiff), [], "all");
m.max_pitch_rad = max(abs(d.theta_B_rad));
m.final_pitch_rad = d.theta_B_rad(end);
m.final_x_m = d.x_B_m(end);
m.final_dx_mps = d.dx_B_mps(end);
m.max_z_drift_m = max(abs(d.z_B_m - d.z_B_m(1)));
m.final_z_drift_m = d.z_B_m(end) - d.z_B_m(1);
m.velocity_mae_mps = mean(abs(d.dx_B_mps - d.dx_ref_mps));
m.wheel_peak_delta_m = max(abs(d.xi_m - baseNmpc.model.xiEq));
m.wheel_tracking_rms_m = rms(d.xi_m - d.xi_ref_m);
m.nmpc_cpu_max_s = max(d.nmpc_cpu_s);
dt = diff(d.time_s);
validDt = dt > 0;
dw = diff(d{:, 20:22});
if any(validDt)
    m.wrench_slew_rms = rms(dw(validDt, :) ./ dt(validDt), "all");
end
dxSmooth = smoothdata(d.dx_B_mps, "movmean", ...
    max(3, round(0.05 / median(dt(validDt)))));
m.realized_peak_speed_mps = max(abs(d.dx_B_mps));
m.realized_peak_accel_mps2 = max(abs(gradient(dxSmooth, d.time_s)));

robotMass = base.m + 2*(leg.m1 + leg.m2 + leg.mw);
contactFz = d.contact_L_Fz_N + d.contact_R_Fz_N;
m.contact_fz_relative_error = abs(mean(contactFz) - robotMass*base.g) ...
    / (robotMass*base.g);
m.body_fz_relative_error = abs(mean(d.qp_Fz_N) - baseNmpc.model.m*base.g) ...
    / (baseNmpc.model.m*base.g);

guard = m.all_finite && m.nmpc_status_zero_ratio == 1 ...
    && m.nmpc_fault_zero_ratio == 1 && m.qp_feasible_ratio > 0.99 ...
    && m.max_dynamics_residual < 1e-6;
switch c.experiment
    case "stand"
        m.accepted = guard && m.max_pitch_rad < deg2rad(0.5) ...
            && abs(m.final_pitch_rad) < deg2rad(0.1) ...
            && m.max_z_drift_m < 0.005 ...
            && m.contact_fz_relative_error < 0.02 ...
            && m.body_fz_relative_error < 0.02;
    case "pitch"
        band = max(deg2rad(0.1), 0.02*abs(c.x0(3)));
        m.settling_time_s = settlingTime(d.time_s, abs(d.theta_B_rad) <= band, 0);
        tail = d.time_s >= d.time_s(end) - 1;
        m.steady_pitch_error_rad = mean(abs(d.theta_B_rad(tail)));
        m.accepted = guard && isfinite(m.settling_time_s) ...
            && abs(m.final_pitch_rad) < deg2rad(0.1);
    case {"velocity", "lqr_sensitivity", "nmpc_sensitivity"}
        m.accepted = guard && m.velocity_mae_mps < 0.1 ...
            && m.max_pitch_rad < deg2rad(5);
    case "constraint"
        m.accepted = guard && m.velocity_mae_mps < 0.15 ...
            && m.max_pitch_rad < deg2rad(5);
    case "disturbance"
        before = d.time_s < 2.5;
        x0 = mean(d.x_B_m(before));
        m.disturbance_max_displacement_m = max(abs(d.x_B_m - x0));
        recovered = abs(d.theta_B_rad) < deg2rad(0.1) ...
            & abs(d.dx_B_mps) < 0.02;
        m.disturbance_recovery_time_s = settlingTime(d.time_s, recovered, 3.0);
        m.accepted = guard && isfinite(m.disturbance_recovery_time_s);
end

positive = d.ax_ref_mps2 > 0.05*max(d.ax_ref_mps2);
if any(positive)
    xi0 = baseNmpc.model.xiEq;
    ideal = d.xi_ideal_delta_m(positive);
    ref = d.xi_ref_m(positive) - xi0;
    actual = d.xi_m(positive) - xi0;
    a = d.ax_ref_mps2(positive);
    m.nmp_ideal_direction_ratio = mean(ideal .* a < 0);
    m.nmp_ref_direction_ratio = mean(ref .* a < 0);
    m.nmp_actual_direction_ratio = mean(actual .* a < 0);
    m.nmp_ideal_peak_m = min(ideal);
    m.nmp_ref_peak_m = min(ref);
    m.nmp_actual_peak_m = min(actual);
    if abs(m.nmp_ideal_peak_m) > 1e-9
        m.nmp_actual_to_ideal_ratio = ...
            abs(m.nmp_actual_peak_m / m.nmp_ideal_peak_m);
    end
end
end

function m = emptyMetrics()
m = struct("case_name", "", "experiment", "", "parameter", "", ...
    "sim_ok", false, "accepted", false, "error_message", "", ...
    "command_v_mps", NaN, "command_a_mps2", NaN, "mu", NaN, ...
    "tau_scale", NaN, "all_finite", false, ...
    "nmpc_status_zero_ratio", NaN, "nmpc_fault_zero_ratio", NaN, ...
    "qp_feasible_ratio", NaN, "max_dynamics_residual", NaN, ...
    "max_slack_norm", NaN, "max_torque_Nm", NaN, ...
    "torque_saturation_ratio", NaN, "max_symmetry_difference", NaN, ...
    "max_pitch_rad", NaN, "final_pitch_rad", NaN, "final_x_m", NaN, ...
    "final_dx_mps", NaN, "max_z_drift_m", NaN, "final_z_drift_m", NaN, ...
    "velocity_mae_mps", NaN, "wheel_peak_delta_m", NaN, ...
    "wheel_tracking_rms_m", NaN, "nmpc_cpu_max_s", NaN, ...
    "wrench_slew_rms", NaN, "realized_peak_speed_mps", NaN, ...
    "realized_peak_accel_mps2", NaN, "contact_fz_relative_error", NaN, ...
    "body_fz_relative_error", NaN, "settling_time_s", NaN, ...
    "steady_pitch_error_rad", NaN, "disturbance_max_displacement_m", NaN, ...
    "disturbance_recovery_time_s", NaN, "nmp_ideal_direction_ratio", NaN, ...
    "nmp_ref_direction_ratio", NaN, "nmp_actual_direction_ratio", NaN, ...
    "nmp_ideal_peak_m", NaN, "nmp_ref_peak_m", NaN, ...
    "nmp_actual_peak_m", NaN, "nmp_actual_to_ideal_ratio", NaN);
end

function tSettle = settlingTime(t, inBand, startTime)
tSettle = NaN;
for idx = find(t >= startTime, 1):numel(t)
    if all(inBand(idx:end))
        tSettle = t(idx) - startTime;
        return;
    end
end
end

function values = atTime(logs, name, t, discrete)
[sourceTime, data] = signalRows(logs, name);
if numel(sourceTime) == 1
    values = repmat(data, numel(t), 1);
    return;
end
method = "linear";
if discrete
    method = "previous";
end
[sourceTime, uniqueIdx] = unique(sourceTime, "stable");
data = data(uniqueIdx, :);
values = interp1(sourceTime, data, t, method, "extrap");
end

function [time, data] = signalRows(logs, name)
element = logs.get(name);
assert(~isempty(element), "Missing logsout signal: %s", name);
values = element.Values;
time = double(values.Time(:));
data = squeeze(double(values.Data));
if isvector(data)
    data = data(:);
elseif size(data, 1) ~= numel(time) && size(data, 2) == numel(time)
    data = data.';
end
end

function clearFunctions()
clear base_nmpc_command base_nmpc_reference ...
    coupled_two_leg_qp_core common_mode_qp_validation_signal ...
    wheel_position_lqr_reference
end

function restoreModel(oldDir, model, initFcn, qpBlock, qpFcn, qpWidth, splitBlock, splitOutputs)
if bdIsLoaded(model)
    try
        set_param(model, "InitFcn", initFcn);
        set_param(qpBlock, "MATLABFcn", qpFcn, "OutputDimensions", qpWidth);
        set_param(splitBlock, "Outputs", splitOutputs);
    catch
    end
    close_system(model, 0);
end
cd(oldDir);
end

function ensureFolder(path)
if ~isfolder(path)
    mkdir(path);
end
end

function value = tag(x)
value = replace(string(sprintf("%.3g", x)), ".", "p");
end

function value = demandTag(profile)
value = "v" + tag(profile(1)) + "_a" + tag(profile(2));
end
