function results = test_full_two_leg_planar_simulink()
%TEST_FULL_TWO_LEG_PLANAR_SIMULINK Validate x/z/pitch on the 6-DoF plant.

model = "source";
load_system(model);
initFcn = get_param(model, "InitFcn");
wasDirty = get_param(model, "Dirty");
cleanup = onCleanup(@() restoreModel(model, initFcn, wasDirty));
set_param(model, "InitFcn", "");

cases = [
    struct("name", "stand", "tracking", "stand", "stopTime", 5, "pitch", 0)
    struct("name", "z", "tracking", "z", "stopTime", 10, "pitch", 0)
    struct("name", "velocity", "tracking", "velocity", "stopTime", 10, "pitch", 0)
    struct("name", "pitch", "tracking", "stand", "stopTime", 5, ...
        "pitch", deg2rad(2))
];
results = repmat(struct(), numel(cases), 1);

for k = 1:numel(cases)
    evalin("base", "startup");
    set_initial_base_state([0; 0; cases(k).pitch; 0; 0; 0]);
    configure_base_tracking_case(cases(k).tracking, "lqr", model);
    clear coupled_two_leg_qp_core base_nmpc_command wheel_position_governor_step
    out = sim(model, "StopTime", string(cases(k).stopTime), ...
        "ReturnWorkspaceOutputs", "on");
    logs = out.logsout;

    [time, state] = namedSignal(logs, "baseWheelState");
    [~, qp] = namedSignal(logs, "coupledQpSignal");
    [~, symmetry] = namedSignal(logs, "symmetryLegState");
    [~, angularVelocity] = namedSignal(logs, "baseAngularVelocity");
    [~, lateralState] = namedSignal(logs, "baseLateralState");
    [~, nmpcStatus] = namedSignal(logs, "nmpcStatus");
    [~, nmpcFault] = namedSignal(logs, "nmpcFault");

    qDelta = 0.5 * (symmetry(:, 1:3) - symmetry(:, 7:9));
    dqDelta = 0.5 * (symmetry(:, 4:6) - symmetry(:, 10:12));
    results(k).caseName = cases(k).name;
    results(k).finalState = state(end, 1:6);
    results(k).maxAbsPitchDeg = rad2deg(max(abs(state(:, 3))));
    results(k).pitchSettlingTime = settlingTime( ...
        time, state(:, 3), deg2rad(0.1));
    results(k).maxAbsQDelta = max(abs(qDelta), [], 1);
    results(k).maxAbsDqDelta = max(abs(dqDelta), [], 1);
    results(k).maxAbsXiDelta = max(abs(qp(:, 40)));
    results(k).maxAbsDxiDelta = max(abs(qp(:, 41)));
    results(k).maxAbsLateralState = max(abs(lateralState), [], 1);
    results(k).maxAbsRollYawRate = max(abs(angularVelocity(:, 1:2)), [], 1);
    results(k).qpFeasibleRatio = mean(qp(:, 14) > 0.5);
    results(k).minExitFlag = min(qp(:, 19));
    results(k).maxDynamicsResidual = max(qp(:, 20));
    results(k).minFrictionMargin = min(qp(:, 32:33), [], "all");
    results(k).minTorqueMargin = min(qp(:, 34:39), [], "all");
    results(k).nmpcStatus = unique(nmpcStatus).';
    results(k).nmpcFaultRatio = mean(nmpcFault ~= 0);

    assert(results(k).qpFeasibleRatio > 0.99, ...
        "%s QP feasible ratio fell below 99%%.", cases(k).name);
    assert(results(k).maxDynamicsResidual < 1e-6, ...
        "%s dynamics residual exceeded 1e-6.", cases(k).name);
    assert(all(nmpcStatus == 0) && all(nmpcFault == 0), ...
        "%s NMPC reported a status or fault.", cases(k).name);
    assert(results(k).minFrictionMargin >= -1e-6, ...
        "%s violated a friction margin.", cases(k).name);
    assert(results(k).minTorqueMargin >= -1e-6, ...
        "%s violated a torque margin.", cases(k).name);

    fprintf(["%s: final [x z pitch] = [%.4g %.4g %.4g deg], " + ...
        "QP %.2f%%, dyn %.3g, max |xiDelta| %.3g m.\n"], ...
        cases(k).name, ...
        state(end, 1), state(end, 2), rad2deg(state(end, 3)), ...
        100*results(k).qpFeasibleRatio, results(k).maxDynamicsResidual, ...
        results(k).maxAbsXiDelta);
end
clear cleanup
end

function [time, data] = namedSignal(logs, name)
element = logs.get(name);
assert(~isempty(element), "Missing logged signal %s.", name);
time = element.Values.Time(:);
data = squeeze(element.Values.Data);
if isvector(data)
    data = data(:);
elseif size(data, 1) ~= numel(time) && size(data, 2) == numel(time)
    data = data.';
end
end

function value = settlingTime(time, signal, band)
outside = find(abs(signal) > band, 1, "last");
if isempty(outside)
    value = 0;
elseif outside == numel(time)
    value = inf;
else
    value = time(outside + 1);
end
end

function restoreModel(model, initFcn, wasDirty)
if bdIsLoaded(model)
    set_param(model, "InitFcn", initFcn, "Dirty", wasDirty);
end
end
