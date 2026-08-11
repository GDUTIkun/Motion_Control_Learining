function summary = test_turning_motion_simulink(caseFilter)
%TEST_TURNING_MOTION_SIMULINK Run T1--T6 steering validation and sweep.

if nargin < 1 || isempty(caseFilter)
    caseFilter = "all";
end
caseFilter = string(caseFilter);
model = "source";
evalin("base", "startup");
load_system(model);
initFcn = get_param(model, "InitFcn");
wasDirty = get_param(model, "Dirty");
cleanup = onCleanup(@() restoreModel(model, initFcn, wasDirty));
set_param(model, "InitFcn", "");

cases = [
    steeringCase("T1_low_small", "T1", 0.05, 0.03, "single", 2.0)
    steeringCase("T2_left", "T2_T3", 0.10, 0.05, "single", 2.0)
    steeringCase("T2_right", "T2_T3", 0.10, -0.05, "single", 2.0)
    steeringCase("T4_rate_0p03", "T4", 0.10, 0.03, "single", 2.0)
    steeringCase("T4_rate_0p08", "T4", 0.10, 0.08, "single", 2.0)
    steeringCase("T5_vx_0p05", "T5", 0.05, 0.05, "single", 2.0)
    steeringCase("T5_vx_0p15", "T5", 0.15, 0.05, "single", 2.0)
    steeringCase("T6_s_curve", "T6", 0.10, 0.04, "s", 1.0)
];
if caseFilter ~= "all"
    selected = contains(string({cases.name}), caseFilter, ...
        "IgnoreCase", true);
    cases = cases(selected);
    if isempty(cases)
        error("test_turning_motion_simulink:UnknownFilter", ...
            "No steering case matches '%s'.", caseFilter);
    end
end

records = repmat(emptyRecord(), numel(cases), 1);
for k = 1:numel(cases)
    set_initial_base_state(zeros(6, 1), zeros(4, 1));
    configure_turning_case(cases(k).vx, cases(k).yawRate, ...
        cases(k).mode, model);
    base = evalin("base", "base");
    baseLqr = evalin("base", "baseLqr");
    trajectory = base.trajectory;
    trajectory.cruiseDuration = 5.0;
    trajectory.turning.startTime = 2.2;
    trajectory.turning.rampDuration = 0.5;
    trajectory.turning.holdDuration = cases(k).holdDuration;
    trajectory.turning.zeroHoldDuration = 0.5;
    base.trajectory = trajectory;
    baseLqr.trajectory = trajectory;
    assignin("base", "base", base);
    assignin("base", "baseLqr", baseLqr);

    clear spatial_two_leg_qp_core coupled_two_leg_qp_core ...
        full_base_nmpc_command wheel_position_governor_step
    simulationStart = tic;
    out = sim(model, "StopTime", "9", "ReturnWorkspaceOutputs", "on");
    simulationWallTime = toc(simulationStart);
    logs = out.logsout;

    [time, state] = namedSignal(logs, "baseNmpcState");
    [symmetryTime, symmetry] = namedSignal(logs, "symmetryLegState");
    [~, qp] = namedSignal(logs, "coupledQpSignal");
    [wrenchTime, nmpcWrench] = namedSignal(logs, "nmpcBodyWrench");
    [~, nmpcStatus] = namedSignal(logs, "nmpcStatus");
    [~, nmpcCpuTime] = namedSignal(logs, "nmpcCpuTime");
    [~, nmpcFault] = namedSignal(logs, "nmpcFault");

    reference = zeros(numel(time), 6);
    vxReference = zeros(numel(time), 1);
    for i = 1:numel(time)
        [baseReference, ~] = floating_base_reference(time(i), baseLqr);
        vxReference(i) = baseReference(4);
        reference(i, :) = turning_motion_reference(time(i), ...
            vxReference(i), trajectory, evalin("base", ...
            "fullBaseNmpc.model.halfTrack")).';
    end
    wheelReference = zeros(numel(symmetryTime), 2);
    for i = 1:numel(symmetryTime)
        [baseReference, ~] = floating_base_reference(symmetryTime(i), baseLqr);
        turning = turning_motion_reference(symmetryTime(i), ...
            baseReference(4), trajectory, evalin("base", ...
            "fullBaseNmpc.model.halfTrack"));
        wheelReference(i, :) = turning(5:6).';
    end
    wrenchYawRateReference = zeros(numel(wrenchTime), 1);
    for i = 1:numel(wrenchTime)
        [baseReference, ~] = floating_base_reference(wrenchTime(i), baseLqr);
        turning = turning_motion_reference(wrenchTime(i), ...
            baseReference(4), trajectory, evalin("base", ...
            "fullBaseNmpc.model.halfTrack"));
        wrenchYawRateReference(i) = turning(2);
    end
    leg = evalin("base", "leg");
    wheelSpeed = -leg.r*[sum(symmetry(:, 4:6), 2), ...
        sum(symmetry(:, 10:12), 2)];

    active = abs(reference(:, 2)) >= 0.8*abs(cases(k).yawRate);
    if ~any(active)
        active = abs(reference(:, 2)) > 0;
    end
    wheelDifference = wheelSpeed(:, 2) - wheelSpeed(:, 1);
    wheelReferenceDifference = wheelReference(:, 2) - wheelReference(:, 1);
    wheelActive = abs(wheelReferenceDifference) ...
        >= 0.8*max(abs(wheelReferenceDifference));
    primaryWheelActive = wheelReferenceDifference*sign(cases(k).yawRate) ...
        >= 0.8*max(abs(wheelReferenceDifference));
    wrenchActive = wrenchYawRateReference*sign(cases(k).yawRate) ...
        >= 0.8*abs(cases(k).yawRate);
    exitWindow = time >= profileEndTime(trajectory.turning) + 0.5;
    xiDelta = 0.5*(state(:, 13) - state(:, 14));
    dxiDelta = 0.5*(state(:, 15) - state(:, 16));
    yawRateError = state(:, 12) - reference(:, 2);
    yawError = state(:, 6) - reference(:, 1);
    actualWheelDifference = mean(wheelDifference(primaryWheelActive));
    referenceWheelDifference = mean( ...
        wheelReferenceDifference(primaryWheelActive));
    primaryActive = reference(:, 2)*sign(cases(k).yawRate) ...
        >= 0.8*abs(cases(k).yawRate);
    meanYawRate = mean(state(primaryActive, 12));
    meanVx = mean(state(primaryActive, 7));
    actualRadius = safeRatio(meanVx, meanYawRate);
    referenceRadius = safeRatio(mean(vxReference(primaryActive)), ...
        mean(reference(primaryActive, 2)));

    record = emptyRecord();
    record.name = string(cases(k).name);
    record.testGroup = string(cases(k).testGroup);
    record.vxRef = cases(k).vx;
    record.yawRateRef = cases(k).yawRate;
    record.yawRateRmse = rms(yawRateError(active));
    record.yawRmseDeg = rad2deg(rms(yawError(active)));
    record.vxRmse = rms(state(active, 7) - vxReference(active));
    record.actualRadius = actualRadius;
    record.referenceRadius = referenceRadius;
    record.maxAbsLateralPosition = max(abs(state(:, 2)));
    record.maxAbsLateralVelocity = max(abs(state(:, 8)));
    record.maxAbsYawDeg = rad2deg(max(abs(state(:, 6))));
    record.finalYawDeg = rad2deg(state(end, 6));
    record.actualLeftWheelSpeed = mean(wheelSpeed(primaryWheelActive, 1));
    record.actualRightWheelSpeed = mean(wheelSpeed(primaryWheelActive, 2));
    record.referenceLeftWheelSpeed = mean( ...
        wheelReference(primaryWheelActive, 1));
    record.referenceRightWheelSpeed = mean( ...
        wheelReference(primaryWheelActive, 2));
    record.actualWheelSpeedDifference = actualWheelDifference;
    record.referenceWheelSpeedDifference = referenceWheelDifference;
    record.wheelDirectionCorrect = mean( ...
        wheelDifference(wheelActive).*wheelReferenceDifference(wheelActive) ...
        > 0) > 0.95;
    record.meanLongitudinalWrenchDifference = mean( ...
        nmpcWrench(wrenchActive, 7) - nmpcWrench(wrenchActive, 1));
    record.maxAbsLongitudinalWrenchDifference = max(abs( ...
        nmpcWrench(:, 7) - nmpcWrench(:, 1)));
    record.maxAbsVerticalWrenchDifference = max(abs( ...
        nmpcWrench(:, 9) - nmpcWrench(:, 3)));
    contactForceDifference = abs(qp(:, 33:35) - qp(:, 36:38));
    record.maxAbsContactForceDifferenceX = max(contactForceDifference(:, 1));
    record.maxAbsContactForceDifferenceY = max(contactForceDifference(:, 2));
    record.maxAbsContactForceDifferenceZ = max(contactForceDifference(:, 3));
    record.maxAbsRollDeg = rad2deg(max(abs(state(:, 4))));
    record.maxAbsPitchDeg = rad2deg(max(abs(state(:, 5))));
    record.maxAbsXiDelta = max(abs(xiDelta));
    record.finalXiDelta = xiDelta(end);
    record.finalDxiDelta = dxiDelta(end);
    record.finalAbsYawRate = max(abs(state(exitWindow, 12)));
    record.maxWrenchSlackNorm = max(qp(:, 31));
    record.maxRollingResidual = max(abs(qp(:, 49)));
    record.maxLateralResidual = max(abs(qp(:, 50)));
    record.maxNormalResidual = max(abs(qp(:, 51)));
    record.qpFeasibleRatio = mean(qp(:, 32) > 0.5);
    record.maxDynamicsResidual = max(qp(:, 40));
    record.maxWrenchResidual = max(qp(:, 42));
    record.minFrictionMargin = min(qp(:, 61:64), [], "all");
    record.minTorqueMargin = min(qp(:, 65:70), [], "all");
    record.nmpcStatusMax = max(abs(nmpcStatus));
    record.nmpcFaultRatio = mean(nmpcFault ~= 0);
    record.maxNmpcSolveTimeMs = 1e3*max(nmpcCpuTime);
    record.maxQpSolveTimeMs = 1e3*max(qp(:, 54));
    record.simulationWallTime = simulationWallTime;
    [record.stable, record.failureReason] = classify(record);
    records(k) = record;
    fprintf("%s: stable=%d, yaw-rate RMSE %.4g, vx RMSE %.4g, " + ...
        "wheel diff %.4g/%.4g m/s, max |xiDelta| %.4g m, " + ...
        "roll/pitch %.3g/%.3g deg, contact %.3g/%.3g/%.3g.\n", ...
        record.name, record.stable, record.yawRateRmse, record.vxRmse, ...
        record.actualWheelSpeedDifference, ...
        record.referenceWheelSpeedDifference, record.maxAbsXiDelta, ...
        record.maxAbsRollDeg, record.maxAbsPitchDeg, ...
        record.maxRollingResidual, record.maxLateralResidual, ...
        record.maxNormalResidual);
end

summary = struct2table(records);
if caseFilter == "all"
    mirrorRows = ismember(summary.name, ["T2_left", "T2_right"]);
    mirror = summary(mirrorRows, :);
    if height(mirror) == 2
        mirrorYawRateError = abs(mirror.yawRateRmse(1) ...
            - mirror.yawRateRmse(2));
        mirrorXiPeakError = abs(mirror.maxAbsXiDelta(1) ...
            - mirror.maxAbsXiDelta(2));
        mirrorWheelDifferenceError = abs( ...
            abs(mirror.actualWheelSpeedDifference(1)) ...
            - abs(mirror.actualWheelSpeedDifference(2)));
        summary.mirrorYawRateRmseDifference = nan(height(summary), 1);
        summary.mirrorXiPeakDifference = nan(height(summary), 1);
        summary.mirrorWheelDifferenceMagnitudeError = nan(height(summary), 1);
        summary.mirrorYawRateRmseDifference(mirrorRows) = mirrorYawRateError;
        summary.mirrorXiPeakDifference(mirrorRows) = mirrorXiPeakError;
        summary.mirrorWheelDifferenceMagnitudeError(mirrorRows) = ...
            mirrorWheelDifferenceError;
        assert(mirrorYawRateError < 5e-4 ...
            && mirrorXiPeakError < 5e-4 ...
            && mirrorWheelDifferenceError < 2e-3, ...
            "Left/right steering cases are not sufficiently mirrored.");
    end
    writetable(summary, "turning_motion_regression.csv");
    fprintf("Steering sweep: %d/%d cases met every strict criterion.\n", ...
        nnz(summary.stable), height(summary));
end
clear cleanup
end

function data = steeringCase(name, testGroup, vx, yawRate, mode, holdDuration)
data = struct("name", name, "testGroup", testGroup, "vx", vx, ...
    "yawRate", yawRate, "mode", mode, "holdDuration", holdDuration);
end

function record = emptyRecord()
record = struct("name", "", "testGroup", "", "vxRef", 0, ...
    "yawRateRef", 0, "yawRateRmse", 0, "yawRmseDeg", 0, ...
    "vxRmse", 0, "actualRadius", 0, "referenceRadius", 0, ...
    "maxAbsLateralPosition", 0, "maxAbsLateralVelocity", 0, ...
    "maxAbsYawDeg", 0, "finalYawDeg", 0, ...
    "actualLeftWheelSpeed", 0, "actualRightWheelSpeed", 0, ...
    "referenceLeftWheelSpeed", 0, "referenceRightWheelSpeed", 0, ...
    "actualWheelSpeedDifference", 0, ...
    "referenceWheelSpeedDifference", 0, "wheelDirectionCorrect", false, ...
    "meanLongitudinalWrenchDifference", 0, ...
    "maxAbsLongitudinalWrenchDifference", 0, ...
    "maxAbsVerticalWrenchDifference", 0, ...
    "maxAbsContactForceDifferenceX", 0, ...
    "maxAbsContactForceDifferenceY", 0, ...
    "maxAbsContactForceDifferenceZ", 0, ...
    "maxAbsRollDeg", 0, ...
    "maxAbsPitchDeg", 0, "maxAbsXiDelta", 0, "finalXiDelta", 0, ...
    "finalDxiDelta", 0, "finalAbsYawRate", 0, ...
    "maxWrenchSlackNorm", 0, "maxRollingResidual", 0, ...
    "maxLateralResidual", 0, "maxNormalResidual", 0, ...
    "qpFeasibleRatio", 0, "maxDynamicsResidual", 0, ...
    "maxWrenchResidual", 0, "minFrictionMargin", 0, ...
    "minTorqueMargin", 0, "nmpcStatusMax", 0, ...
    "nmpcFaultRatio", 0, "maxNmpcSolveTimeMs", 0, ...
    "maxQpSolveTimeMs", 0, "simulationWallTime", 0, ...
    "stable", false, "failureReason", "");
end

function [stable, reason] = classify(record)
failures = strings(0, 1);
if record.qpFeasibleRatio < 0.99
    failures(end + 1) = "QP feasibility";
end
if record.nmpcStatusMax ~= 0 || record.nmpcFaultRatio ~= 0
    failures(end + 1) = "NMPC status/fault";
end
if record.maxDynamicsResidual >= 1e-6 || record.maxWrenchResidual >= 1e-6
    failures(end + 1) = "dynamics/wrench residual";
end
if record.minFrictionMargin < -1e-6 || record.minTorqueMargin < -1e-6
    failures(end + 1) = "friction/torque margin";
end
if record.yawRateRmse > max(0.015, 0.35*abs(record.yawRateRef))
    failures(end + 1) = "yaw-rate tracking";
end
if record.vxRmse > max(0.03, 0.5*abs(record.vxRef))
    failures(end + 1) = "longitudinal tracking";
end
if ~record.wheelDirectionCorrect
    failures(end + 1) = "wheel differential direction";
end
if record.maxAbsRollDeg > 2 || record.maxAbsPitchDeg > 3
    failures(end + 1) = "attitude bound";
end
if record.maxAbsXiDelta > 0.08 || abs(record.finalXiDelta) > 0.03
    failures(end + 1) = "xi_delta bound/recovery";
end
if record.finalAbsYawRate > 0.02
    failures(end + 1) = "turn exit";
end
if record.maxWrenchSlackNorm > 0.05 ...
        || max([record.maxRollingResidual, record.maxLateralResidual, ...
        record.maxNormalResidual]) > 5
    failures(end + 1) = "wrench/contact residual";
end
stable = isempty(failures);
if stable
    reason = "ok";
else
    reason = strjoin(failures, "; ");
end
end

function ratio = safeRatio(numerator, denominator)
if abs(denominator) < 1e-9
    ratio = sign(numerator)*inf;
else
    ratio = numerator/denominator;
end
end

function endTime = profileEndTime(turning)
startTime = turning.startTime;
rampDuration = turning.rampDuration;
holdDuration = turning.holdDuration;
if lower(string(turning.mode)) == "s"
    endTime = startTime + 4*rampDuration + 2*holdDuration ...
        + turning.zeroHoldDuration;
else
    endTime = startTime + 2*rampDuration + holdDuration;
end
end

function [time, data] = namedSignal(logs, name)
element = logs.get(name);
assert(~isempty(element), "Missing logged signal %s.", name);
time = element.Values.Time;
data = squeeze(element.Values.Data);
if isvector(data)
    data = data(:);
elseif size(data, 1) ~= numel(time)
    data = data.';
end
end

function restoreModel(model, initFcn, wasDirty)
if bdIsLoaded(model)
    set_param(model, "InitFcn", initFcn);
    if wasDirty == "off"
        set_param(model, "Dirty", "off");
    end
end
end
