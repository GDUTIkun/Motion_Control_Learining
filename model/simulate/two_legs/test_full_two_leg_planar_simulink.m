function results = test_full_two_leg_planar_simulink()
%TEST_FULL_TWO_LEG_PLANAR_SIMULINK Validate the spatial QP on the 6-DoF plant.

model = "source";
evalin("base", "startup");
load_system(model);
initFcn = get_param(model, "InitFcn");
wasDirty = get_param(model, "Dirty");
cleanup = onCleanup(@() restoreModel(model, initFcn, wasDirty));
set_param(model, "InitFcn", "");

cases = [
    struct("name", "stand", "tracking", "stand", "stopTime", 5, ...
        "pitch", 0, "roll", 0, "yaw", 0)
    struct("name", "z", "tracking", "z", "stopTime", 10, ...
        "pitch", 0, "roll", 0, "yaw", 0)
    struct("name", "velocity", "tracking", "velocity", "stopTime", 10, ...
        "pitch", 0, "roll", 0, "yaw", 0)
    struct("name", "pitch", "tracking", "stand", "stopTime", 5, ...
        "pitch", deg2rad(2), "roll", 0, "yaw", 0)
    struct("name", "roll", "tracking", "stand", "stopTime", 5, ...
        "pitch", 0, "roll", deg2rad(2), "yaw", 0)
    struct("name", "yaw", "tracking", "stand", "stopTime", 5, ...
        "pitch", 0, "roll", 0, "yaw", deg2rad(2))
];
results = repmat(struct(), numel(cases), 1);

for k = 1:numel(cases)
    set_initial_base_state([0; 0; cases(k).pitch; 0; 0; 0], ...
        [cases(k).roll; cases(k).yaw; 0; 0]);
    configure_base_tracking_case(cases(k).tracking, "lqr", model);
    clear coupled_two_leg_qp_core full_base_nmpc_command ...
        wheel_position_governor_step
    out = sim(model, "StopTime", string(cases(k).stopTime), ...
        "ReturnWorkspaceOutputs", "on");
    logs = out.logsout;

    [time, state] = namedSignal(logs, "baseWheelState");
    [~, qp] = namedSignal(logs, "coupledQpSignal");
    [~, symmetry] = namedSignal(logs, "symmetryLegState");
    [~, angularVelocity] = namedSignal(logs, "baseAngularVelocity");
    [attitudeTime, rollYaw] = namedSignal(logs, "baseRollYawState");
    [~, lateralState] = namedSignal(logs, "baseLateralState");
    [~, fullNmpcState] = namedSignal(logs, "baseNmpcState");
    [~, nmpcWrench] = namedSignal(logs, "nmpcBodyWrench");
    [~, nmpcStatus] = namedSignal(logs, "nmpcStatus");
    [~, nmpcCpuTime] = namedSignal(logs, "nmpcCpuTime");
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
    results(k).finalXiDelta = qp(end, 71);
    results(k).maxAbsXiDelta = max(abs(qp(:, 71)));
    results(k).maxAbsDxiDelta = max(abs(qp(:, 72)));
    results(k).maxContactAccelNorm = max(qp(:, 41));
    results(k).maxAbsContactForceDifference = max( ...
        abs(qp(:, 33:35) - qp(:, 36:38)), [], 1);
    results(k).maxAbsLateralState = max(abs(lateralState), [], 1);
    results(k).maxAbsRollYawRate = max(abs(angularVelocity(:, 1:2)), [], 1);
    results(k).finalRollYawDeg = rad2deg(rollYaw(end, 1:2));
    results(k).maxAbsRollYawDeg = rad2deg(max(abs(rollYaw(:, 1:2)), [], 1));
    results(k).rollSettlingTime = settlingTime( ...
        attitudeTime, rollYaw(:, 1), deg2rad(0.1));
    results(k).yawSettlingTime = settlingTime( ...
        attitudeTime, rollYaw(:, 2), deg2rad(0.1));
    results(k).qpFeasibleRatio = mean(qp(:, 32) > 0.5);
    results(k).minExitFlag = min(qp(:, 39));
    results(k).maxDynamicsResidual = max(qp(:, 40));
    results(k).maxWrenchResidual = max(qp(:, 42));
    results(k).minFrictionMargin = min(qp(:, 61:64), [], "all");
    results(k).minTorqueMargin = min(qp(:, 65:70), [], "all");
    results(k).nmpcStatus = unique(nmpcStatus).';
    results(k).meanNmpcCpuTime = mean(nmpcCpuTime);
    results(k).maxNmpcCpuTime = max(nmpcCpuTime);
    results(k).nmpcFaultRatio = mean(nmpcFault ~= 0);
    assert(size(fullNmpcState, 2) == 16 && size(nmpcWrench, 2) == 12, ...
        "%s did not use the 16-state/12-input full NMPC.", cases(k).name);

    assert(results(k).qpFeasibleRatio > 0.99, ...
        "%s QP feasible ratio fell below 99%%.", cases(k).name);
    assert(results(k).maxDynamicsResidual < 1e-6, ...
        "%s dynamics residual exceeded 1e-6.", cases(k).name);
    assert(isfinite(results(k).maxContactAccelNorm), ...
        "%s contact acceleration is nonfinite.", cases(k).name);
    assert(results(k).maxWrenchResidual < 1e-6, ...
        "%s wrench residual exceeded 1e-6.", cases(k).name);
    assert(all(nmpcStatus == 0) && all(nmpcFault == 0), ...
        "%s NMPC reported a status or fault.", cases(k).name);
    assert(results(k).minFrictionMargin >= -1e-6, ...
        "%s violated a friction margin.", cases(k).name);
    assert(results(k).minTorqueMargin >= -1e-6, ...
        "%s violated a torque margin.", cases(k).name);
    assert(results(k).maxNmpcCpuTime <= evalin("base", ...
        "fullBaseNmpc.maxSolveTime"), ...
        "%s NMPC exceeded its supervisory deadline.", cases(k).name);
    if cases(k).roll ~= 0
        assert(abs(results(k).finalRollYawDeg(1)) < 0.2, ...
            "The roll disturbance did not recover below 0.2 deg.");
    end
    if cases(k).yaw ~= 0
        assert(abs(results(k).finalRollYawDeg(2)) < 0.2, ...
            "The yaw disturbance did not recover below 0.2 deg.");
    end

    fprintf(["%s: final [x z pitch] = [%.4g %.4g %.4g deg], " + ...
        "roll/yaw = [%.4g %.4g] deg, QP %.2f%%, dyn %.3g, " + ...
        "contact accel %.3g, max |FcL-FcR| %.3g N, " + ...
        "max |xiDelta| %.3g m, NMPC max %.3g ms.\n"], ...
        cases(k).name, ...
        state(end, 1), state(end, 2), rad2deg(state(end, 3)), ...
        results(k).finalRollYawDeg(1), results(k).finalRollYawDeg(2), ...
        100*results(k).qpFeasibleRatio, results(k).maxDynamicsResidual, ...
        results(k).maxContactAccelNorm, ...
        max(results(k).maxAbsContactForceDifference), ...
        results(k).maxAbsXiDelta, 1e3*results(k).maxNmpcCpuTime);
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
