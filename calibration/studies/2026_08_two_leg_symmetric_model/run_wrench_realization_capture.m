function result = run_wrench_realization_capture(stopTime, initialBaseState, ...
        pulseFx, pulseDuration)
%RUN_WRENCH_REALIZATION_CAPTURE Compare requested, QP, and physical Fx.

if nargin < 1 || isempty(stopTime)
    stopTime = 3;
end
if nargin < 2 || isempty(initialBaseState)
    initialBaseState = zeros(6, 1);
end
initialBaseState = double(initialBaseState(:));
assert(numel(initialBaseState) == 6 && all(isfinite(initialBaseState)), ...
    "initialBaseState must contain six finite values.");
if nargin < 3 || isempty(pulseFx)
    pulseFx = 0;
end
pulseFx = double(pulseFx);
assert(isscalar(pulseFx) && isfinite(pulseFx));
if nargin < 4 || isempty(pulseDuration)
    pulseDuration = 0.05;
end
pulseDuration = double(pulseDuration);
assert(isscalar(pulseDuration) && isfinite(pulseDuration) ...
    && pulseDuration > 0);

studyDir = fileparts(mfilename("fullpath"));
codeRoot = fileparts(fileparts(fileparts(studyDir)));
calibrationDir = fullfile(codeRoot, "calibration");
modelDir = fullfile(codeRoot, "model", "simulate", "two_legs");
processedDir = fullfile(calibrationDir, "data", "processed");
figureDir = fullfile(calibrationDir, "figures", "studies", ...
    "2026_08_two_leg_symmetric_model");
resultDir = fullfile(calibrationDir, "results", ...
    "2026_08_two_leg_symmetric_model", "wrench_realization_capture");
ensureFolder(processedDir);
ensureFolder(figureDir);
ensureFolder(resultDir);

addpath(modelDir, studyDir);
oldFolder = cd(modelDir);
load_system("source");
initFcn = get_param("source", "InitFcn");
cleanup = onCleanup(@() finishRun(oldFolder, initFcn));
set_param("source", "InitFcn", "");

evalin("base", "run('" + replace(fullfile(modelDir, "startup.m"), ...
    "'", "''") + "')");
configure_symmetric_two_leg_simulink(false);
configure_base_tracking_case("stand", "lqr", "source");
set_initial_base_state(initialBaseState);
if pulseFx ~= 0
    wrenchPulse = struct("fxBody", pulseFx, "startTime", 1.0, ...
        "duration", pulseDuration);
    assignin("base", "wrenchPulse", wrenchPulse);
    upperController = find_system("source/PD_only", "SearchDepth", 1, ...
        "BlockType", "MATLABFcn", ...
        "MATLABFcn", "floating_base_lqr_command");
    assert(isscalar(upperController));
    set_param(upperController{1}, ...
        "MATLABFcn", "floating_base_lqr_pulse_command");
end
clear floating_base_lqr_wrench coupled_two_leg_qp_core wheel_position_lqr_reference

simulation = sim("source", "StopTime", string(stopTime), ...
    "ReturnWorkspaceOutputs", "on");
logs = simulation.logsout;
base = evalin("base", "base");
t = (0:base.Ts:stopTime).';

[commonTime, common] = loggedRows(logs, "commonWheelStateSignal");
[commandTime, upperCommand] = loggedRows(logs, "totalUpperCommand");
[qpTime, coupledQp] = loggedRows(logs, "coupledQpSignal");

common = interp1(commonTime, common, t, "linear", "extrap");
upperCommand = interp1(commandTime, upperCommand, t, "previous", "extrap");
coupledQp = interp1(qpTime, coupledQp, t, "previous", "extrap");

desiredFx = -upperCommand(:, 1);
qpFx = coupledQp(:, 10);
qpSlackFx = coupledQp(:, 7);
qpContactFx = coupledQp(:, 15) + coupledQp(:, 17);
qpFeasible = coupledQp(:, 14) > 0.5;
x = common(:, 2);
dx = common(:, 5);
ax = gradient(dx, base.Ts);
actualFx = base.m * ax;
actualFxFiltered = movmean(actualFx, 21);

capture = table(t, x, dx, ax, desiredFx, qpFx, qpSlackFx, ...
    qpContactFx, actualFx, actualFxFiltered, qpFeasible, ...
    'VariableNames', ["time_s", "x_base_m", "dx_base_mps", ...
    "ax_base_mps2", "fx_desired_N", "fx_qp_feasible_N", ...
    "fx_qp_slack_N", "fx_qp_contact_N", "fx_actual_from_base_N", ...
    "fx_actual_filtered_N", "qp_feasible"]);
assert(all(isfinite(capture{:,:}), "all"), ...
    "Wrench realization capture contains non-finite values.");

steady = t >= max(0, stopTime - 0.5);
metrics = struct( ...
    "initialBaseState", initialBaseState.', ...
    "pulseFx", pulseFx, ...
    "pulseDuration", pulseDuration, ...
    "finalX", x(end), ...
    "finalDx", dx(end), ...
    "meanDesiredFxLastHalfSecond", mean(desiredFx(steady)), ...
    "meanQpFxLastHalfSecond", mean(qpFx(steady)), ...
    "meanQpContactFxLastHalfSecond", mean(qpContactFx(steady)), ...
    "meanActualFxLastHalfSecond", mean(actualFx(steady)), ...
    "qpVsDesiredRmsLastHalfSecond", ...
        rms(qpFx(steady) - desiredFx(steady)), ...
    "actualVsQpRmsLastHalfSecond", ...
        rms(actualFx(steady) - qpFx(steady)), ...
    "fullRunQpVsDesiredRms", rms(qpFx - desiredFx), ...
    "fullRunActualVsQpRms", rms(actualFx - qpFx), ...
    "fullRunFilteredActualVsQpRms", ...
        rms(actualFxFiltered - qpFx), ...
    "maxAbsDesiredFx", max(abs(desiredFx)), ...
    "maxAbsQpFx", max(abs(qpFx)), ...
    "maxAbsActualFx", max(abs(actualFx)), ...
    "qpFeasibleRatio", mean(qpFeasible), ...
    "maxAbsQpSlackFx", max(abs(qpSlackFx)));

csvFile = fullfile(processedDir, "two_leg_wrench_realization_capture.csv");
matFile = fullfile(resultDir, "wrench_realization_capture.mat");
figureFile = fullfile(figureDir, "wrench_realization_capture.png");
writetable(capture, csvFile);
save(matFile, "capture", "metrics", "stopTime", "initialBaseState", ...
    "pulseFx", "pulseDuration");
plotCapture(capture, figureFile);

result = struct("metrics", metrics, "csvFile", string(csvFile), ...
    "matFile", string(matFile), "figureFile", string(figureFile));
disp(metrics);
clear cleanup
end

function [time, data] = loggedRows(logs, name)
values = logs.get(name).Values;
time = double(values.Time(:));
data = squeeze(double(values.Data));
if isvector(data)
    data = data(:);
elseif size(data, 1) ~= numel(time) && size(data, 2) == numel(time)
    data = data.';
end
end

function plotCapture(capture, figureFile)
f = figure("Visible", "off", "Color", "white", ...
    "Position", [100, 100, 960, 620]);
cleanup = onCleanup(@() close(f));
tiledlayout(2, 1, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(capture.time_s, capture.fx_desired_N, "LineWidth", 1.4);
hold on;
plot(capture.time_s, capture.fx_qp_feasible_N, "--", "LineWidth", 1.4);
plot(capture.time_s, capture.fx_qp_contact_N, ":", "LineWidth", 1.4);
plot(capture.time_s, capture.fx_actual_from_base_N, "LineWidth", 1.2);
plot(capture.time_s, capture.fx_actual_filtered_N, "-.", "LineWidth", 1.0);
yline(0, ":", "Color", [0.45, 0.45, 0.45]);
ylabel("Horizontal body force (N)");
legend("LQR requested", "QP feasible", "QP contact F_c", ...
    "actual from m_B a_x", "actual, 105 ms mean", ...
    "Location", "best");
title("Two-leg horizontal wrench realization");
grid on;

nexttile;
yyaxis left;
plot(capture.time_s, capture.x_base_m, "LineWidth", 1.4);
ylabel("x_B (m)");
yyaxis right;
plot(capture.time_s, capture.dx_base_mps, "LineWidth", 1.4);
ylabel("dx_B (m/s)");
xlabel("Time (s)");
grid on;
exportgraphics(f, figureFile, "Resolution", 180);
clear cleanup
end

function ensureFolder(folder)
if ~isfolder(folder)
    mkdir(folder);
end
end

function finishRun(oldFolder, initFcn)
if bdIsLoaded("source")
    set_param("source", "InitFcn", initFcn);
    close_system("source", 0);
end
cd(oldFolder);
end
