function results = run_standing_cases(stopTime)
%RUN_STANDING_CASES Run nominal stand and a small pitch disturbance.

if nargin < 1 || isempty(stopTime)
    stopTime = 3;
end

studyDir = fileparts(mfilename("fullpath"));
codeRoot = fileparts(fileparts(fileparts(studyDir)));
modelDir = fullfile(codeRoot, "model", "simulate", "two_legs");
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

cases = struct( ...
    "name", {"stand", "pitch_2deg"}, ...
    "x0", {zeros(6, 1), [0; 0; deg2rad(2); 0; 0; 0]});
for i = 1:numel(cases)
    set_initial_base_state(cases(i).x0);
    clear floating_base_lqr_wrench controller_qp_core wheel_position_lqr_reference
    output = sim("source", "StopTime", string(stopTime), ...
        "ReturnWorkspaceOutputs", "on");
    results.(cases(i).name) = metrics(output.logsout);
end

ctrl = evalin("base", "ctrl");
stand = results.stand;
pitch = results.pitch_2deg;
results.stopTime = stopTime;
results.pass = commonChecks(stand, ctrl) && commonChecks(pitch, ctrl) ...
    && stand.maxAbsPitch < deg2rad(5) ...
    && abs(stand.finalBaseState(3)) < deg2rad(1) ...
    && max(abs(stand.finalBaseState(4:6))) < 0.1 ...
    && pitch.maxAbsPitch < deg2rad(15) ...
    && abs(pitch.finalBaseState(3)) < deg2rad(1) ...
    && abs(pitch.finalBaseState(3)) < 0.5*deg2rad(2) ...
    && max(abs(pitch.finalBaseState(4:6))) < 0.15;

fprintf("Stand metrics:\n");
disp(stand);
fprintf("Pitch-disturbance metrics:\n");
disp(pitch);
fprintf("Overall pass: %d\n", results.pass);
assert(results.pass, ...
    "The symmetric two-leg LQR-QP standing validation did not pass.");
clear cleanup
end

function value = commonChecks(data, ctrl)
value = data.allFinite ...
    && all(data.maxAbsTorque <= ctrl.tauMax(:).' + 1e-8) ...
    && data.qpFeasibleRatio >= 0.99 ...
    && data.maxLegStateDifference < 1e-6 ...
    && data.maxTorqueDifference < 1e-6 ...
    && data.maxAbsBasePosition(1) < 0.5 ...
    && data.maxAbsBasePosition(2) < 0.1;
end

function data = metrics(logs)
common = samples(logs, "commonWheelStateSignal");
legState = samples(logs, "symmetryLegState");
leftQp = samples(logs, "leftQpSignal");
rightQp = samples(logs, "rightQpSignal");

baseState = common(:, 2:7);
stateDifference = legState(:, 1:6) - legState(:, 7:12);
torqueDifference = leftQp(:, 1:3) - rightQp(:, 1:3);
data = struct( ...
    "finalBaseState", baseState(end, :), ...
    "maxAbsBasePosition", max(abs(baseState(:, 1:2)), [], 1), ...
    "maxAbsPitch", max(abs(baseState(:, 3))), ...
    "maxAbsTorque", max(abs([leftQp(:, 1:3); rightQp(:, 1:3)]), [], 1), ...
    "maxLegStateDifference", max(abs(stateDifference), [], "all"), ...
    "maxTorqueDifference", max(abs(torqueDifference), [], "all"), ...
    "maxSlackNorm", max([leftQp(:, 10); rightQp(:, 10)]), ...
    "qpFeasibleRatio", mean([leftQp(:, 11); rightQp(:, 11)]), ...
    "allFinite", all(isfinite(common), "all") ...
        && all(isfinite(legState), "all") ...
        && all(isfinite(leftQp), "all") ...
        && all(isfinite(rightQp), "all"));
end

function data = samples(logs, name)
values = logs.get(name).Values;
data = squeeze(values.Data);
if isvector(data)
    data = data(:);
elseif size(data, 1) ~= numel(values.Time) && ...
        size(data, 2) == numel(values.Time)
    data = data.';
end
end

function finishRun(oldFolder, initFcn)
if bdIsLoaded("source")
    set_param("source", "InitFcn", initFcn);
    close_system("source", 0);
end
cd(oldFolder);
end
