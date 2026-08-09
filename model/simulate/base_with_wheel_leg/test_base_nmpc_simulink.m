function results = test_base_nmpc_simulink(stopTime, testFallbacks)
%TEST_BASE_NMPC_SIMULINK Exercise nominal and LQR-fallback Simulink paths.

if nargin < 1 || isempty(stopTime)
    stopTime = 0.25;
end
if nargin < 2 || isempty(testFallbacks)
    testFallbacks = true;
end

model = "source";
load_system(model);
initFcn = get_param(model, "InitFcn");
wasDirty = get_param(model, "Dirty");
cleanup = onCleanup(@() restoreModel(model, initFcn, wasDirty));
set_param(model, "InitFcn", "");
evalin("base", "startup");
configure_base_tracking_case("velocity", "lqr");
baseNmpc = evalin("base", "baseNmpc");
baseLqr = evalin("base", "baseLqr");
ctrl = evalin("base", "ctrl");

nominal = runCase(model, baseNmpc, stopTime);
assert(all(nominal.status == 0), "NMPC returned a nonzero status.");
assert(all(isfinite(nominal.bodyWrench), "all"));
assert(all(isfinite(nominal.cpuTime)));
assert(all(nominal.cpuTime <= baseNmpc.maxSolveTime));
assert(all(nominal.fallback == 0), "Nominal NMPC unexpectedly fell back.");
assert(all(isfinite(nominal.wheelState), "all") ...
    && all(isfinite(nominal.wheelReference), "all"));
assert(all(nominal.wheelState(:, 7) >= baseNmpc.xiMin - 1e-9) ...
    && all(nominal.wheelState(:, 7) <= baseNmpc.xiMax + 1e-9));
assertTorqueBounds(nominal.torque, ctrl.tauMax);
nominalRms = trackingRms(nominal, baseLqr);

if ~testFallbacks
    results = nominalResults(nominal, nominalRms);
    fprintf("Nominal Base NMPC check passed; RMS [x theta] %s.\n", ...
        mat2str(nominalRms, 6));
    clear cleanup
    return;
end

manualConfig = baseNmpc;
manualConfig.enabled = false;
manual = runCase(model, manualConfig, stopTime);
assert(all(manual.fallback == 1), "Manual fallback was not active.");
assertCommandMatchesLqr(manual);

baselineRms = trackingRms(manual, baseLqr);
trajectory = baseLqr.trajectory;
roundTripEnd = trajectory.settleTime + 2*(trajectory.accelDuration ...
    + trajectory.cruiseDuration + trajectory.decelDuration) ...
    + trajectory.turnHoldDuration;
if stopTime >= roundTripEnd
    assert(max(abs(nominal.wheelState(:, 2))) < 0.15, ...
        "NMPC base height diverged during the full trajectory.");
    assert(max(abs(nominal.wheelState(:, 3))) < deg2rad(30), ...
        "NMPC base pitch diverged during the full trajectory.");
    assert(abs(nominal.wheelState(end, 3)) < 0.05 ...
        && max(abs(nominal.wheelState(end, 4:6))) < 0.05, ...
        "NMPC did not settle after the full trajectory.");
    fprintf("Full trajectory RMS [x theta]: NMPC %s, LQR %s.\n", ...
        mat2str(nominalRms, 6), mat2str(baselineRms, 6));
    assert(all(nominalRms <= 1.05*baselineRms + 1e-9), ...
        "NMPC position/pitch RMS exceeds 105%% of the LQR baseline.");
end

timeoutConfig = baseNmpc;
timeoutConfig.maxSolveTime = 0;
timeout = runCase(model, timeoutConfig, stopTime);
assert(all(timeout.fallback == 1), "Timeout fallback was not active.");
assertCommandMatchesLqr(timeout);

results = struct( ...
    "maxCpuTime", max(nominal.cpuTime), ...
    "status", unique(nominal.status).', ...
    "maxAbsTorque", max(abs(nominal.torque), [], 1), ...
    "nominalPositionPitchRms", nominalRms, ...
    "baselinePositionPitchRms", baselineRms, ...
    "nominalFallbackRatio", mean(nominal.fallback), ...
    "manualFallbackRatio", mean(manual.fallback), ...
    "timeoutFallbackRatio", mean(timeout.fallback));
fprintf("Base NMPC Simulink checks passed; max CPU time %.6g s.\n", ...
    results.maxCpuTime);
clear cleanup
end

function results = nominalResults(nominal, nominalRms)
results = struct( ...
    "maxCpuTime", max(nominal.cpuTime), ...
    "status", unique(nominal.status).', ...
    "maxAbsTorque", max(abs(nominal.torque), [], 1), ...
    "nominalPositionPitchRms", nominalRms, ...
    "nominalFallbackRatio", mean(nominal.fallback));
end

function data = runCase(model, config, stopTime)
assignin("base", "baseNmpc", config);
clear base_nmpc_command floating_base_lqr_command
out = sim(model, "StopTime", num2str(stopTime, "%.15g"), ...
    "ReturnWorkspaceOutputs", "on");
logs = out.logsout;
data.status = namedData(logs, "nmpcStatus");
data.cpuTime = namedData(logs, "nmpcCpuTime");
data.fallback = namedData(logs, "nmpcFallback");
data.bodyWrench = namedData(logs, "nmpcBodyWrench");
data.wheelState = namedData(logs, "baseWheelState");
data.wheelReference = namedData(logs, "wheelPositionLqrReference");
[qpSignal, ~] = pathData(logs, 10, "source/Interpreted MATLAB Function");
data.torque = qpSignal(:, 1:3);
data.qpWrenchSlack = qpSignal(:, 4:6);
data.qpWrenchFeasible = qpSignal(:, 7:9);
data.qpWrenchSlackNorm = qpSignal(:, 10);
[data.lqrCommand, data.lqrTime] = pathData(logs, 3, ...
    "source/PD_only/Interpreted MATLAB Function");
[data.controllerInput, data.controllerTime] = pathData(logs, 20, ...
    "source/PD_only/Mux1");
[data.baseState, data.baseTime] = pathData(logs, 7, "source/PD_only/Mux");
end

function data = namedData(logs, name)
element = logs.get(name);
data = sampleRows(element.Values);
end

function [data, time] = pathData(logs, width, path)
for i = 1:logs.numElements
    element = logs.get(i);
    data = sampleRows(element.Values);
    blockPath = element.BlockPath.convertToCell;
    if size(data, 2) == width && string(blockPath{1}) == path
        time = element.Values.Time(:);
        return;
    end
end
error("test_base_nmpc_simulink:MissingLog", ...
    "Could not find logged signal %s.", path);
end

function data = sampleRows(values)
data = squeeze(values.Data);
if isvector(data)
    data = data(:);
elseif size(data, 1) ~= numel(values.Time) && size(data, 2) == numel(values.Time)
    data = data.';
end
end

function assertTorqueBounds(torque, tauMax)
assert(all(abs(torque) <= tauMax(:).' + 1e-8, "all"), ...
    "The lower QP exceeded a joint torque limit.");
end

function assertCommandMatchesLqr(data)
selected = data.controllerInput(:, 14:16);
lqr = interp1(data.lqrTime, data.lqrCommand, data.controllerTime, ...
    "previous", "extrap");
assert(max(abs(selected - lqr), [], "all") < 1e-9, ...
    "Fallback output does not match the original LQR branch.");
end

function value = trackingRms(data, baseLqr)
state = data.baseState(:, 2:7);
reference = zeros(size(state));
for i = 1:numel(data.baseTime)
    reference(i, :) = floating_base_reference(data.baseTime(i), baseLqr).';
end
error = state - reference;
value = sqrt(mean(error(:, [1, 3]).^2, 1));
end

function restoreModel(model, initFcn, wasDirty)
if bdIsLoaded(model)
    set_param(model, "InitFcn", initFcn, "Dirty", wasDirty);
end
end
