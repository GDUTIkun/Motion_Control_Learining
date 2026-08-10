function result = run_wheel_position_capture(stopTime)
%RUN_WHEEL_POSITION_CAPTURE Capture one forward-acceleration wheel-planning case.

if nargin < 1 || isempty(stopTime)
    stopTime = 2.0;
end

studyDir = fileparts(mfilename("fullpath"));
codeRoot = fileparts(fileparts(fileparts(studyDir)));
calibrationDir = fullfile(codeRoot, "calibration");
modelDir = fullfile(codeRoot, "model", "simulate", "two_legs");
resultDir = fullfile(calibrationDir, "results", ...
    "2026_08_two_leg_symmetric_model", "wheel_position_capture");
figureDir = fullfile(calibrationDir, "figures", "studies", ...
    "2026_08_two_leg_symmetric_model");
processedDir = fullfile(calibrationDir, "data", "processed");
ensureFolder(resultDir);
ensureFolder(figureDir);
ensureFolder(processedDir);

addpath(modelDir);
oldFolder = cd(modelDir);
load_system("source");
initFcn = get_param("source", "InitFcn");
cleanup = onCleanup(@() finishRun(oldFolder, initFcn));
set_param("source", "InitFcn", "");

evalin("base", "run('" + replace(fullfile(modelDir, "startup.m"), ...
    "'", "''") + "')");
configure_symmetric_two_leg_simulink(false);
configure_base_tracking_case("velocity", "lqr", "source");
set_initial_base_state(zeros(6, 1));
clear floating_base_lqr_wrench controller_qp_core wheel_position_lqr_reference

simulation = sim("source", "StopTime", string(stopTime), ...
    "ReturnWorkspaceOutputs", "on");
logs = simulation.logsout;
[time, common] = loggedRows(logs, "commonWheelStateSignal");
[referenceTime, wheelReference] = loggedRows(logs, "commonWheelReference");
assert(size(common, 2) >= 10 && size(wheelReference, 2) >= 1, ...
    "Expected common wheel state and reference logs are missing columns.");

baseLqr = evalin("base", "baseLqr");
wheelLqr = evalin("base", "wheelLqr");
xiReference = interp1(referenceTime, wheelReference(:, 1), time, ...
    "linear", "extrap");
xiActual = common(:, 8);
height = common(:, 10);
aXReference = zeros(size(time));
for idx = 1:numel(time)
    [~, acceleration] = floating_base_reference(time(idx), baseLqr);
    aXReference(idx) = acceleration(1);
end
xiFeedforward = -height ./ baseLqr.model.g .* aXReference;

capture = table(time, aXReference, xiFeedforward, xiReference, xiActual, ...
    common(:, 2), common(:, 5), height, ...
    'VariableNames', ["time_s", "ax_ref_mps2", "xi_ff_m", "xi_ref_m", ...
    "xi_actual_m", "x_base_m", "dx_base_mps", "height_m"]);

active = aXReference > 0.05 * max(aXReference);
assert(any(active) && all(isfinite(capture{:,:}), "all"), ...
    "The capture does not contain a finite forward-acceleration segment.");
neutral = wheelLqr.neutral;
activeStart = find(active, 1, "first");
baselineIndex = max(1, activeStart - 1);
metrics = struct( ...
    "peakAxReference", max(aXReference(active)), ...
    "peakTheoreticalBackshift", neutral - min(xiFeedforward(active)), ...
    "peakPlannedBackshift", neutral - min(xiReference(active)), ...
    "peakActualBackshift", neutral - min(xiActual(active)), ...
    "plannedAtAccelerationStart", xiReference(baselineIndex), ...
    "actualAtAccelerationStart", xiActual(baselineIndex), ...
    "accelerationInducedPlannedBackshift", ...
        xiReference(baselineIndex) - min(xiReference(active)), ...
    "accelerationInducedActualBackshift", ...
        xiActual(baselineIndex) - min(xiActual(active)), ...
    "baseXAtAccelerationStart", common(baselineIndex, 2), ...
    "plannerCorrectionRms", rms(xiReference(active) - xiFeedforward(active)), ...
    "actualTrackingRms", rms(xiActual(active) - xiReference(active)), ...
    "plannedOppositeAccelerationRatio", ...
        mean((xiReference(active) - neutral) .* aXReference(active) < 0), ...
    "actualOppositeAccelerationRatio", ...
        mean((xiActual(active) - neutral) .* aXReference(active) < 0));

csvFile = fullfile(processedDir, "two_leg_wheel_position_capture.csv");
matFile = fullfile(resultDir, "wheel_position_capture.mat");
figureFile = fullfile(figureDir, "wheel_position_capture.png");
writetable(capture, csvFile);
save(matFile, "capture", "metrics", "stopTime");
plotCapture(capture, figureFile);

result = struct("metrics", metrics, "csvFile", string(csvFile), ...
    "matFile", string(matFile), "figureFile", string(figureFile));
disp(metrics);
fprintf("Saved wheel-position capture to:\n%s\n", csvFile);
clear cleanup
end

function ensureFolder(folder)
if ~isfolder(folder)
    mkdir(folder);
end
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
figureHandle = figure("Visible", "off", "Color", "white", ...
    "Position", [100, 100, 960, 620]);
cleanup = onCleanup(@() close(figureHandle));
tiledlayout(2, 1, "TileSpacing", "compact", "Padding", "compact");

nexttile;
plot(capture.time_s, capture.ax_ref_mps2, "LineWidth", 1.5);
yline(0, ":", "Color", [0.45, 0.45, 0.45]);
ylabel("a_x^{ref} (m/s^2)");
title("Two-leg common-mode wheel-position planning capture");
grid on;

nexttile;
plot(capture.time_s, capture.xi_ff_m, "--", "LineWidth", 1.5);
hold on;
plot(capture.time_s, capture.xi_ref_m, "LineWidth", 1.5);
plot(capture.time_s, capture.xi_actual_m, "LineWidth", 1.5);
yline(0, ":", "Color", [0.45, 0.45, 0.45]);
xlabel("Time (s)");
ylabel("Relative wheel position \xi (m)");
legend("theoretical \xi_{ff}", "planned \xi_{ref}", ...
    "actual \xi", "Location", "best");
grid on;
exportgraphics(figureHandle, figureFile, "Resolution", 180);
clear cleanup
end

function finishRun(oldFolder, initFcn)
if bdIsLoaded("source")
    set_param("source", "InitFcn", initFcn);
    close_system("source", 0);
end
cd(oldFolder);
end
