function result = run_wrench_realization_capture(stopTime)
%RUN_WRENCH_REALIZATION_CAPTURE Compare requested, QP, and physical Fx.

if nargin < 1 || isempty(stopTime)
    stopTime = 3;
end

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

addpath(modelDir);
oldFolder = cd(modelDir);
load_system("source");
initFcn = get_param("source", "InitFcn");
cleanup = onCleanup(@() finishRun(oldFolder, initFcn));
set_param("source", "InitFcn", "");

evalin("base", "run('" + replace(fullfile(modelDir, "startup.m"), ...
    "'", "''") + "')");
configure_symmetric_two_leg_simulink(false);
configure_base_tracking_case("stand", "lqr");
set_initial_base_state(zeros(6, 1));
clear floating_base_lqr_wrench controller_qp_core wheel_position_lqr_reference

simulation = sim("source", "StopTime", string(stopTime), ...
    "ReturnWorkspaceOutputs", "on");
logs = simulation.logsout;
base = evalin("base", "base");
t = (0:base.Ts:stopTime).';

[commonTime, common] = loggedRows(logs, "commonWheelStateSignal");
[commandTime, perLegCommand] = loggedRows(logs, "perLegWrenchCommand");
[leftTime, leftQp] = loggedRows(logs, "leftQpSignal");
[rightTime, rightQp] = loggedRows(logs, "rightQpSignal");

common = interp1(commonTime, common, t, "linear", "extrap");
perLegCommand = interp1(commandTime, perLegCommand, t, "previous", "extrap");
leftQp = interp1(leftTime, leftQp, t, "previous", "extrap");
rightQp = interp1(rightTime, rightQp, t, "previous", "extrap");

desiredFx = -2 * perLegCommand(:, 1);
qpFx = leftQp(:, 7) + rightQp(:, 7);
qpSlackFx = leftQp(:, 4) + rightQp(:, 4);
qpContactFx = leftQp(:, 12) + rightQp(:, 12);
x = common(:, 2);
dx = common(:, 5);
ax = gradient(dx, base.Ts);
actualFx = base.m * movmean(ax, 21);

capture = table(t, x, dx, ax, desiredFx, qpFx, qpSlackFx, ...
    qpContactFx, actualFx, ...
    'VariableNames', ["time_s", "x_base_m", "dx_base_mps", ...
    "ax_base_mps2", "fx_desired_N", "fx_qp_feasible_N", ...
    "fx_qp_slack_N", "fx_qp_contact_N", "fx_actual_from_base_N"]);
assert(all(isfinite(capture{:,:}), "all"), ...
    "Wrench realization capture contains non-finite values.");

steady = t >= max(0, stopTime - 0.5);
metrics = struct( ...
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
    "maxAbsQpSlackFx", max(abs(qpSlackFx)));

csvFile = fullfile(processedDir, "two_leg_wrench_realization_capture.csv");
matFile = fullfile(resultDir, "wrench_realization_capture.mat");
figureFile = fullfile(figureDir, "wrench_realization_capture.png");
writetable(capture, csvFile);
save(matFile, "capture", "metrics", "stopTime");
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
yline(0, ":", "Color", [0.45, 0.45, 0.45]);
ylabel("Horizontal body force (N)");
legend("LQR requested", "QP feasible", "QP contact F_c", ...
    "actual from m_B a_x", ...
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
