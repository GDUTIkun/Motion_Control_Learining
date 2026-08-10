function results = test_base_nmpc_simulink(stopTime, testFaultHold, caseMode)
%TEST_BASE_NMPC_SIMULINK Exercise direct NMPC on the strict common plant.

if nargin < 1 || isempty(stopTime)
    stopTime = 0.25;
end
if nargin < 2 || isempty(testFaultHold)
    testFaultHold = true;
end
if nargin < 3 || isempty(caseMode)
    caseMode = "stand";
end

model = "source_common";
load_system(model);
initFcn = get_param(model, "InitFcn");
wasDirty = get_param(model, "Dirty");
cleanup = onCleanup(@() restoreModel(model, initFcn, wasDirty));
set_param(model, "InitFcn", "");
evalin("base", "startup");
configure_base_tracking_case(caseMode, "lqr", model);
baseNmpc = evalin("base", "baseNmpc");
ctrl = evalin("base", "ctrl");

nominal = runCase(model, baseNmpc, stopTime);
assert(all(nominal.status == 0), "NMPC returned a nonzero status.");
assert(all(isfinite(nominal.bodyWrench), "all"));
assert(all(isfinite(nominal.cpuTime)));
assert(all(nominal.fault == 0), "Nominal NMPC command was rejected.");
assert(all(isfinite(nominal.wheelState), "all") ...
    && all(isfinite(nominal.wheelReference), "all"));
assert(all(nominal.wheelState(:, 7) >= baseNmpc.xiMin - 1e-9) ...
    && all(nominal.wheelState(:, 7) <= baseNmpc.xiMax + 1e-9));
assertTorqueBounds(nominal.qpSignal(:, 1:6), ctrl.tauMax);
assert(max(abs(nominal.wheelState(:, 3))) < deg2rad(30), ...
    "Base pitch diverged under direct NMPC.");

faultRatio = 0;
if testFaultHold
    disabled = baseNmpc;
    disabled.enabled = false;
    held = runCase(model, disabled, stopTime);
    assert(all(held.fault(2:end) == 1), ...
        "Disabled NMPC did not activate hold mode after initialization.");
    equilibrium = [-baseNmpc.model.uEq(1:2); baseNmpc.model.uEq(3)].';
    assert(max(abs(held.selectedCommand - equilibrium), [], "all") < 1e-9, ...
        "NMPC hold mode did not preserve the equilibrium wrench.");
    faultRatio = mean(held.fault);
end

results = struct( ...
    "maxCpuTime", max(nominal.cpuTime), ...
    "status", unique(nominal.status).', ...
    "maxAbsTorque", max(abs(nominal.qpSignal(:, 1:6)), [], 1), ...
    "maxAbsBaseX", max(abs(nominal.wheelState(:, 1))), ...
    "maxAbsPitch", max(abs(nominal.wheelState(:, 3))), ...
    "finalBaseState", nominal.wheelState(end, 1:6), ...
    "nominalFaultRatio", mean(nominal.fault), ...
    "disabledFaultRatio", faultRatio);
fprintf("Direct NMPC Simulink checks passed; max CPU time %.6g s.\n", ...
    results.maxCpuTime);
clear cleanup
end

function data = runCase(model, config, stopTime)
assignin("base", "baseNmpc", config);
clear base_nmpc_command
out = sim(model, "StopTime", num2str(stopTime, "%.15g"), ...
    "ReturnWorkspaceOutputs", "on");
logs = out.logsout;
data.status = namedData(logs, "nmpcStatus");
data.cpuTime = namedData(logs, "nmpcCpuTime");
data.fault = namedData(logs, "nmpcFault");
data.bodyWrench = namedData(logs, "nmpcBodyWrench");
data.selectedCommand = namedData(logs, "totalUpperCommand");
data.wheelState = namedData(logs, "baseWheelState");
data.wheelReference = namedData(logs, "wheelPositionLqrReference");
data.qpSignal = namedData(logs, "coupledQpSignal");
end

function data = namedData(logs, name)
element = logs.get(name);
assert(~isempty(element), "Missing logged signal %s.", name);
data = squeeze(element.Values.Data);
if isvector(data)
    data = data(:);
elseif size(data, 1) ~= numel(element.Values.Time) ...
        && size(data, 2) == numel(element.Values.Time)
    data = data.';
end
end

function assertTorqueBounds(torque, tauMax)
limits = [tauMax(:); tauMax(:)].';
assert(all(abs(torque) <= limits + 1e-8, "all"), ...
    "The lower QP exceeded a joint torque limit.");
end

function restoreModel(model, initFcn, wasDirty)
if bdIsLoaded(model)
    set_param(model, "InitFcn", initFcn, "Dirty", wasDirty);
end
end
