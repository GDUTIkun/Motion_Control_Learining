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
metrics.zB = xin(:, 3);
metrics.dxB = xin(:, 5);
metrics.dzB = xin(:, 6);
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
[contactVelocityActual, contactVelocityRef] = contactVelocityResiduals(xin, ...
    qAbs, dqAbs, qdAbs, dqdAbs, vars);
metrics.contactVelocityActual = contactVelocityActual;
metrics.contactVelocityRef = contactVelocityRef;
[wheelCenterActual, wheelCenterRef] = wheelCenterPositions(xin, qAbs, vars);
metrics.wheelCenterActual = wheelCenterActual;
metrics.wheelCenterRef = wheelCenterRef;
metrics.wheelCenterError = wheelCenterActual - wheelCenterRef;
metrics.qddCmd = qddCmd;
metrics.qddSol = qddSol;
metrics.qddError = qddSol - qddCmd;
metrics.qpExitflag = qpExitflag;
metrics.contactForce = contactForce;
end

function [actualPosition, refPosition] = wheelCenterPositions(xin, qAbs, vars)
n = size(xin, 1);
actualPosition = zeros(n, 2);
refPosition = zeros(n, 2);

xOHNom = 0;
if isfield(vars.traj, "xO0")
    xOHNom = vars.traj.xO0;
end
groundTop = vars.base.simscapeGroundTopY;
if evalin("base", "exist('hip', 'var')") && evalin("base", "isfield(hip, 'groundTopY')")
    groundTop = evalin("base", "hip.groundTopY");
end

for k = 1:n
    baseState = xin(k, 2:7).';
    pH = floatingHipPosition(baseState, vars.base);
    kin = wheel_leg_kinematics(qAbs(k, :).', zeros(3, 1), [], vars.leg);
    actualPosition(k, :) = (pH + kin.pO).';
    refPosition(k, :) = [pH(1) + xOHNom, groundTop + vars.leg.r];
end
end

function pH = floatingHipPosition(baseState, base)
theta = baseState(3);
rH = rotatePitch2D(base.rHBody(:), theta);
pH = baseState(1:2) + rH;
end

function [actualResidual, refResidual] = contactVelocityResiduals(xin, ...
    qAbs, dqAbs, qdAbs, dqdAbs, vars)
n = size(xin, 1);
actualResidual = zeros(n, 2);
refResidual = zeros(n, 2);

for k = 1:n
    baseState = xin(k, 2:7).';
    vH = floatingHipVelocity(baseState, vars.base);

    kinActual = wheel_leg_kinematics(qAbs(k, :).', dqAbs(k, :).', [], vars.leg);
    actualResidual(k, :) = (kinActual.Jc * dqAbs(k, :).' + vH).';

    kinRef = wheel_leg_kinematics(qdAbs(k, :).', dqdAbs(k, :).', [], vars.leg);
    refResidual(k, :) = (kinRef.Jc * dqdAbs(k, :).' + vH).';
end
end

function vH = floatingHipVelocity(baseState, base)
theta = baseState(3);
dtheta = baseState(6);
rH = rotatePitch2D(base.rHBody(:), theta);
drdtheta = [-rH(2); rH(1)];
vH = baseState(4:5) + dtheta * drdtheta;
end

function rWorld = rotatePitch2D(rBody, theta)
rx0 = rBody(1);
rz0 = rBody(2);
rWorld = [
    cos(theta)*rx0 - sin(theta)*rz0;
    sin(theta)*rx0 + cos(theta)*rz0
];
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
summary.contactVelocityActualRms = rms(metrics.contactVelocityActual);
summary.contactVelocityActualMaxAbs = max(abs(metrics.contactVelocityActual), [], 1);
summary.contactVelocityRefRms = rms(metrics.contactVelocityRef);
summary.contactVelocityRefMaxAbs = max(abs(metrics.contactVelocityRef), [], 1);
summary.wheelCenterFinalError = metrics.wheelCenterError(end, :);
summary.wheelCenterRmsError = rms(metrics.wheelCenterError);
summary.wheelCenterMaxAbsError = max(abs(metrics.wheelCenterError), [], 1);
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
fprintf(fid, "actual contact velocity RMS/maxabs: %s / %s m/s\n", ...
    mat2str(summary.contactVelocityActualRms, 6), ...
    mat2str(summary.contactVelocityActualMaxAbs, 6));
fprintf(fid, "ref contact velocity RMS/maxabs: %s / %s m/s\n", ...
    mat2str(summary.contactVelocityRefRms, 6), ...
    mat2str(summary.contactVelocityRefMaxAbs, 6));
fprintf(fid, "wheel-center final/RMS/maxabs error: %s / %s / %s m\n", ...
    mat2str(summary.wheelCenterFinalError, 6), ...
    mat2str(summary.wheelCenterRmsError, 6), ...
    mat2str(summary.wheelCenterMaxAbsError, 6));
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
plotWheelCenter(metrics, outputDir);
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
tiledlayout(fig, 3, 1);
nexttile;
plot(t, metrics.qddError, "LineWidth", 1.1);
grid on;
ylabel("qddSol-qddCmd");
legend("hip", "knee", "wheel");
nexttile;
plot(metrics.t, metrics.contactVelocityActual, "LineWidth", 1.1);
hold on;
plot(metrics.t, metrics.contactVelocityRef, "--", "LineWidth", 1.1);
grid on;
ylabel("contact v");
legend("actual x", "actual z", "ref x", "ref z");
nexttile;
plot(t, metrics.contactForce, "LineWidth", 1.1);
grid on;
ylabel("Fc");
xlabel("time (s)");
legend("Fcx", "Fcz");
saveas(fig, fullfile(outputDir, "qp_debug.png"));
close(fig);
end

function plotWheelCenter(metrics, outputDir)
labels = ["x", "z"];
fig = figure("Visible", "off", "Name", "wheel center");
tiledlayout(fig, 2, 1);
for i = 1:2
    nexttile;
    plot(metrics.t, metrics.wheelCenterActual(:, i), "LineWidth", 1.1);
    hold on;
    plot(metrics.t, metrics.wheelCenterRef(:, i), "--", "LineWidth", 1.1);
    grid on;
    ylabel("pO " + labels(i) + " (m)");
    legend("actual", "ref");
end
xlabel("time (s)");
saveas(fig, fullfile(outputDir, "wheel_center.png"));
close(fig);

fig = figure("Visible", "off", "Name", "wheel center error");
plot(metrics.t, metrics.wheelCenterError, "LineWidth", 1.1);
grid on;
xlabel("time (s)");
ylabel("pO actual-ref (m)");
legend("x", "z");
saveas(fig, fullfile(outputDir, "wheel_center_error.png"));
close(fig);
end
