function summary = run_diagnostic_case(initialState, stopTime, doConfigureTiming, outputDir)
%RUN_DIAGNOSTIC_CASE Run and summarize one floating-base LQR + QP case.
%
% Examples:
%   run_diagnostic_case
%   run_diagnostic_case(deg2rad(5), 3)
%   run_diagnostic_case(zeros(6,1), 5, false)

if nargin < 1 || isempty(initialState)
    initialState = deg2rad(5);
end
if nargin < 2 || isempty(stopTime)
    stopTime = 3;
end
if nargin < 3 || isempty(doConfigureTiming)
    doConfigureTiming = false;
end
if nargin < 4 || isempty(outputDir)
    stamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    outputDir = fullfile(fileparts(mfilename("fullpath")), "diagnostics", stamp);
end

thisDir = fileparts(mfilename("fullpath"));
oldDir = pwd;
cleanup = onCleanup(@() cd(oldDir));
cd(thisDir);

if ~isfolder(outputDir)
    mkdir(outputDir);
end

evalin("base", "startup");
configure_discrete_controller_timing(doConfigureTiming);
applyInitialState(initialState);

simOut = sim("source", ...
    "StopTime", num2str(stopTime, "%.15g"), ...
    "ReturnWorkspaceOutputs", "on");

logs = simOut.logsout;
signals = extractLoggedSignals(logs);
vars = loadBaseVariables();

metrics = computeMetrics(signals, vars);
summary = makeSummary(metrics, signals, outputDir, initialState, stopTime);

save(fullfile(outputDir, "diagnostic_summary.mat"), "summary", "metrics");
writeSummaryText(summary, fullfile(outputDir, "summary.txt"));
plotDiagnostics(metrics, outputDir);

fprintf("\nDiagnostic summary saved to:\n  %s\n", outputDir);
fprintf("theta final %.6f rad, qAbs RMS %s rad, tau_h/MBy RMS %.6g N*m\n", ...
    summary.thetaFinal, mat2str(summary.qAbsRms, 5), summary.tauMomentRms);
end

function applyInitialState(initialState)
initialState = double(initialState(:));
if isscalar(initialState)
    set_initial_pitch(initialState);
elseif numel(initialState) == 6
    set_initial_base_state(initialState);
else
    error("run_diagnostic_case:InvalidInitialState", ...
        "initialState must be a pitch scalar or a 6-element base state.");
end
end

function vars = loadBaseVariables()
vars = struct();
vars.base = evalin("base", "base");
vars.ctrl = evalin("base", "ctrl");
vars.leg = evalin("base", "leg");
vars.traj = evalin("base", "traj");
end

function signals = extractLoggedSignals(logs)
signals = struct();
signals.tau = getLogMatrix(logs, 3, "source/Interpreted MATLAB");
signals.upperCommand = getLogMatrix(logs, 3, "PD_only/Interpreted MATLAB");
signals.baseMux = getLogMatrix(logs, 7, "PD_only/Mux");
signals.controllerInput = getLogMatrix(logs, 16, "PD_only/Mux1");
end

function signal = getLogMatrix(logs, expectedWidth, blockPathPattern)
signal = [];
for i = 1:logs.numElements
    element = logs.get(i);
    values = element.Values;
    data = squeeze(values.Data);
    if isvector(data)
        width = 1;
    else
        width = size(data, 2);
        if size(data, 1) ~= numel(values.Time) && size(data, 2) == numel(values.Time)
            data = data.';
            width = size(data, 2);
        end
    end

    pathText = "";
    try
        pathCell = element.BlockPath.convertToCell;
        pathText = string(pathCell{1});
    catch
    end

    if width == expectedWidth && contains(pathText, blockPathPattern)
        signal = struct("time", values.Time(:), "data", data, ...
            "blockPath", pathText);
        return;
    end
end

error("run_diagnostic_case:MissingLogSignal", ...
    "Could not find logged signal width %d at block path containing '%s'.", ...
    expectedWidth, blockPathPattern);
end

function metrics = computeMetrics(signals, vars)
xin = signals.controllerInput.data;
t = xin(:, 1);
n = size(xin, 1);

theta = xin(:, 4);
dtheta = xin(:, 7);
qRel = xin(:, 8:10);
dqRel = xin(:, 11:13);
qAbs = qRel;
qAbs(:, 1) = qAbs(:, 1) + theta;
dqAbs = dqRel;
dqAbs(:, 1) = dqAbs(:, 1) + dtheta;

qdAbs = zeros(n, 3);
dqdAbs = zeros(n, 3);
ddqdAbs = zeros(n, 3);
for k = 1:n
    [qd, dqd, ddqd] = floating_base_leg_reference(t(k), xin(k, 2:7).', ...
        vars.traj, vars.leg, vars.base);
    qdAbs(k, :) = qd.';
    dqdAbs(k, :) = dqd.';
    ddqdAbs(k, :) = ddqd.';
end

tau = interp1(signals.tau.time, signals.tau.data, t, "previous", "extrap");
tauRefHip = vars.ctrl.hipMomentToTauSign * xin(:, 16);
tauMomentError = tau(:, 1) - tauRefHip;
tauMax = vars.ctrl.tauMax(:).';
tauSat = abs(tau) >= (tauMax - 1e-8);

[qddCmd, qddSol, qpExitflag, contactForce] = offlineQpDebug(xin, qAbs, ...
    dqAbs, qdAbs, dqdAbs, ddqdAbs, vars);

metrics = struct();
metrics.t = t;
metrics.theta = theta;
metrics.dtheta = dtheta;
metrics.xB = xin(:, 2);
metrics.dxB = xin(:, 5);
metrics.qAbs = qAbs;
metrics.dqAbs = dqAbs;
metrics.qdAbs = qdAbs;
metrics.dqdAbs = dqdAbs;
metrics.qAbsError = qAbs - qdAbs;
metrics.dqAbsError = dqAbs - dqdAbs;
metrics.MByDes = xin(:, 16);
metrics.FHExt = xin(:, 14:15);
metrics.tau = tau;
metrics.tauRefHip = tauRefHip;
metrics.tauMomentError = tauMomentError;
metrics.tauSat = tauSat;
metrics.qddCmd = qddCmd;
metrics.qddSol = qddSol;
metrics.qddError = qddSol - qddCmd;
metrics.qpExitflag = qpExitflag;
metrics.contactForce = contactForce;
end

function [qddCmd, qddSol, qpExitflag, contactForce] = offlineQpDebug(xin, ...
    qAbs, dqAbs, qdAbs, dqdAbs, ddqdAbs, vars)
stride = max(1, floor(size(xin, 1) / 1200));
idx = 1:stride:size(xin, 1);

qddCmd = zeros(numel(idx), 3);
qddSol = zeros(numel(idx), 3);
qpExitflag = zeros(numel(idx), 1);
contactForce = zeros(numel(idx), 2);

clear controller_qp_core
for j = 1:numel(idx)
    k = idx(j);
    [~, debug] = controller_qp_core(xin(k, :).');
    qddSol(j, :) = debug.qdd(:).';
    qpExitflag(j) = debug.exitflag;
    contactForce(j, :) = debug.Fc(:).';
    qddCmd(j, :) = (ddqdAbs(k, :).' ...
        + vars.ctrl.Kd * (dqdAbs(k, :).' - dqAbs(k, :).') ...
        + vars.ctrl.Kp * (qdAbs(k, :).' - qAbs(k, :).')).';
end
end

function summary = makeSummary(metrics, signals, outputDir, initialState, stopTime)
summary = struct();
summary.outputDir = string(outputDir);
summary.initialState = initialState;
summary.stopTime = stopTime;
summary.logBlockPaths = struct( ...
    "tau", signals.tau.blockPath, ...
    "upperCommand", signals.upperCommand.blockPath, ...
    "baseMux", signals.baseMux.blockPath, ...
    "controllerInput", signals.controllerInput.blockPath);
summary.thetaFinal = metrics.theta(end);
summary.thetaMin = min(metrics.theta);
summary.thetaMax = max(metrics.theta);
summary.qAbsFinalError = metrics.qAbsError(end, :);
summary.qAbsRms = rms(metrics.qAbsError);
summary.dqAbsFinalError = metrics.dqAbsError(end, :);
summary.dqAbsRms = rms(metrics.dqAbsError);
summary.tauMomentFinalError = metrics.tauMomentError(end);
summary.tauMomentRms = rms(metrics.tauMomentError);
summary.tauMomentMaxAbs = max(abs(metrics.tauMomentError));
summary.tauMin = min(metrics.tau, [], 1);
summary.tauMax = max(metrics.tau, [], 1);
summary.tauSaturationRatio = mean(metrics.tauSat, 1);
summary.qddErrorRms = rms(metrics.qddError);
summary.qddErrorMaxAbs = max(abs(metrics.qddError), [], 1);
summary.qpExitflags = unique(metrics.qpExitflag).';
end

function writeSummaryText(summary, filePath)
fid = fopen(filePath, "w");
if fid < 0
    error("run_diagnostic_case:CannotWriteSummary", ...
        "Could not write summary file: %s", filePath);
end
closer = onCleanup(@() fclose(fid));

fprintf(fid, "Single wheel-leg diagnostic summary\n");
fprintf(fid, "==================================\n\n");
fprintf(fid, "Output directory: %s\n", summary.outputDir);
fprintf(fid, "Stop time: %.6g s\n\n", summary.stopTime);
fprintf(fid, "theta final/min/max: %.6g / %.6g / %.6g rad\n", ...
    summary.thetaFinal, summary.thetaMin, summary.thetaMax);
fprintf(fid, "qAbs final error: %s rad\n", mat2str(summary.qAbsFinalError, 6));
fprintf(fid, "qAbs RMS error:   %s rad\n", mat2str(summary.qAbsRms, 6));
fprintf(fid, "dqAbs final error: %s rad/s\n", mat2str(summary.dqAbsFinalError, 6));
fprintf(fid, "dqAbs RMS error:   %s rad/s\n", mat2str(summary.dqAbsRms, 6));
fprintf(fid, "tau_h moment RMS/maxabs error: %.6g / %.6g N*m\n", ...
    summary.tauMomentRms, summary.tauMomentMaxAbs);
fprintf(fid, "tau saturation ratio: %s\n", ...
    mat2str(summary.tauSaturationRatio, 6));
fprintf(fid, "qddSol-qddCmd RMS: %s rad/s^2\n", ...
    mat2str(summary.qddErrorRms, 6));
fprintf(fid, "qddSol-qddCmd maxabs: %s rad/s^2\n", ...
    mat2str(summary.qddErrorMaxAbs, 6));
fprintf(fid, "QP exitflags: %s\n", mat2str(summary.qpExitflags));
end

function plotDiagnostics(metrics, outputDir)
plotBase(metrics, outputDir);
plotJointTracking(metrics, outputDir);
plotTorque(metrics, outputDir);
plotQp(metrics, outputDir);
end

function plotBase(metrics, outputDir)
fig = figure("Visible", "off", "Name", "base");
tiledlayout(fig, 2, 1);
nexttile;
plot(metrics.t, metrics.theta, "LineWidth", 1.2);
grid on;
ylabel("thetaB (rad)");
nexttile;
plot(metrics.t, metrics.xB, "LineWidth", 1.2);
hold on;
plot(metrics.t, metrics.dxB, "LineWidth", 1.2);
grid on;
ylabel("xB / dxB");
xlabel("time (s)");
legend("xB", "dxB");
saveas(fig, fullfile(outputDir, "base_state.png"));
close(fig);
end

function plotJointTracking(metrics, outputDir)
names = ["hip abs", "knee", "wheel"];
fig = figure("Visible", "off", "Name", "joint tracking");
tiledlayout(fig, 3, 1);
for i = 1:3
    nexttile;
    plot(metrics.t, metrics.qAbs(:, i), "LineWidth", 1.1);
    hold on;
    plot(metrics.t, metrics.qdAbs(:, i), "--", "LineWidth", 1.1);
    grid on;
    ylabel(names(i));
    legend("actual", "ref");
end
xlabel("time (s)");
saveas(fig, fullfile(outputDir, "joint_tracking.png"));
close(fig);

fig = figure("Visible", "off", "Name", "joint error");
plot(metrics.t, metrics.qAbsError, "LineWidth", 1.1);
grid on;
xlabel("time (s)");
ylabel("qAbs error (rad)");
legend(names);
saveas(fig, fullfile(outputDir, "joint_error.png"));
close(fig);
end

function plotTorque(metrics, outputDir)
fig = figure("Visible", "off", "Name", "torque");
tiledlayout(fig, 2, 1);
nexttile;
plot(metrics.t, metrics.tau(:, 1), "LineWidth", 1.2);
hold on;
plot(metrics.t, metrics.tauRefHip, "--", "LineWidth", 1.2);
grid on;
ylabel("hip torque (N*m)");
legend("tau_h", "hipMomentToTauSign*MBy");
nexttile;
plot(metrics.t, metrics.tau, "LineWidth", 1.1);
grid on;
ylabel("tau (N*m)");
xlabel("time (s)");
legend("hip", "knee", "wheel");
saveas(fig, fullfile(outputDir, "torque.png"));
close(fig);
end

function plotQp(metrics, outputDir)
t = linspace(metrics.t(1), metrics.t(end), size(metrics.qddError, 1));
fig = figure("Visible", "off", "Name", "qp");
tiledlayout(fig, 2, 1);
nexttile;
plot(t, metrics.qddError, "LineWidth", 1.1);
grid on;
ylabel("qddSol-qddCmd");
legend("hip", "knee", "wheel");
nexttile;
plot(t, metrics.contactForce, "LineWidth", 1.1);
grid on;
ylabel("Fc");
xlabel("time (s)");
legend("Fcx", "Fcz");
saveas(fig, fullfile(outputDir, "qp_debug.png"));
close(fig);
end
