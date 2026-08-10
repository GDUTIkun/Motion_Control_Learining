function results = run_validation()
%RUN_VALIDATION Validate the symmetric two-leg common-mode model and wiring.

studyDir = fileparts(mfilename("fullpath"));
codeRoot = fileparts(fileparts(fileparts(studyDir)));
modelDir = fullfile(codeRoot, "model", "simulate", "two_legs");
addpath(modelDir);
oldFolder = cd(modelDir);
cleanup = onCleanup(@() finishValidation(oldFolder));

startupCommand = "run('" + replace(fullfile(modelDir, "startup.m"), ...
    "'", "''") + "')";
evalin("base", startupCommand);
base = evalin("base", "base");
leg = evalin("base", "leg");
ctrl = evalin("base", "ctrl");
traj = evalin("base", "traj");
wheelLqr = evalin("base", "wheelLqr");

expectedIyy = 3 / 12 * (0.45^2 + 0.32^2);
assert(abs(base.Iyy - expectedIyy) < 1e-12);
assert(isequal([base.body.lengthX, base.body.widthY, base.body.heightZ], ...
    [0.45, 0.45, 0.32]));
assert(norm(base.rHBody) < 1e-12);
assert(base.legCount == 2 && base.symmetricLoadShare == 0.5);
assert(abs(base.body.hipPositionBodyLeft3D(2)) == 0.2);
assert(abs(base.body.hipPositionBodyRight3D(2)) == 0.2);

upper = base_wheel_state_space(base, leg, traj);
rollingDenominator = leg.mw * leg.r + leg.Iw / leg.r;
assert(abs(upper.A(6, 3)) < 1e-12);
assert(abs(upper.B(8, 1) - ...
    (-1/base.m - leg.r/(2*rollingDenominator))) < 1e-12);
assert(abs(upper.B(8, 3) + 1/(2*rollingDenominator)) < 1e-12);
commonControllabilityRank = rank(ctrb(upper.A, upper.B), 1e-10);
assert(commonControllabilityRank == 6);
equilibriumDerivative = floating_base_nonlinear_state(0, ...
    base.xEq, upper.uEq, base);
assert(norm(equilibriumDerivative, inf) < 1e-12);

perLegCommand = [0; -base.m*base.g/2; 0];
wheelReference = [wheelLqr.neutral; 0; 0; wheelLqr.neutral];
[qd0, dqd0, ddqd0] = floating_base_leg_reference(0, zeros(6, 1), ...
    traj, leg, base, zeros(2, 1), perLegCommand(1:2), true, ...
    wheelReference);
assert(norm(qd0 - leg.q0, inf) < 1e-10);
assert(norm(dqd0 - leg.dq0, inf) < 1e-10);
assert(norm(ddqd0, inf) < 1e-10);

qLeft = leg.q0 + [0.01; -0.02; 0.03];
dqLeft = leg.dq0 + [0.02; -0.01; 0.04];
qRight = leg.q0 + [-0.015; 0.01; -0.02];
dqRight = leg.dq0 + [-0.01; 0.03; -0.02];
stateA = wheel_position_state_signal([0; zeros(6, 1); ...
    qLeft; dqLeft; qRight; dqRight], base, leg, ctrl);
stateB = wheel_position_state_signal([0; zeros(6, 1); ...
    qRight; dqRight; qLeft; dqLeft], base, leg, ctrl);
assert(norm(stateA - stateB, inf) < 1e-12);

clear coupled_two_leg_qp_core
totalCommand = [0; -base.m*base.g; 0];
qpInput = [0; zeros(6, 1); leg.q0; leg.dq0; leg.q0; leg.dq0; ...
    totalCommand; wheelReference];
[torque, qpDebug] = coupled_two_leg_qp_core(qpInput);
leftTorque = torque(1:3);
rightTorque = torque(4:6);
assert(qpDebug.qpFeasible && all(isfinite(torque)));
assert(max(abs(leftTorque - rightTorque)) < 1e-8);
assert(all(abs(leftTorque) <= ctrl.tauMax + 1e-8));

configure_symmetric_two_leg_simulink(false);
assertPhysicalGeometry();
simulation = sim("source", "StopTime", "0.05", ...
    "ReturnWorkspaceOutputs", "on");
logs = simulation.logsout;
coupledQp = sampleRows(logs.get("coupledQpSignal").Values);
upperCommand = sampleRows(logs.get("totalUpperCommand").Values);
commonWheel = sampleRows(logs.get("commonWheelStateSignal").Values);
assert(all(isfinite(coupledQp), "all"));
assert(all(isfinite(upperCommand), "all"));
assert(all(isfinite(commonWheel), "all"));
assert(all(abs(coupledQp(:, 1:3)) <= ctrl.tauMax(:).' + 1e-8, "all"));
assert(all(abs(coupledQp(:, 4:6)) <= ctrl.tauMax(:).' + 1e-8, "all"));
assert(all(coupledQp(:, 14) > 0.5));
assert(abs(upperCommand(1, 2) + base.m*base.g) < 1e-9);

results = struct( ...
    "Iyy", base.Iyy, ...
    "commonControllabilityRank", commonControllabilityRank, ...
    "maxLeftRightTorqueDifference", ...
        max(abs(coupledQp(:, 1:3) - coupledQp(:, 4:6)), [], "all"), ...
    "initialTotalCommand", upperCommand(1, :), ...
    "samples", size(coupledQp, 1));
disp(results);
clear cleanup
end

function assertPhysicalGeometry()
subsystem = "source/PD_only";
bodyBlock = subsystem + "/Brick Solid";
dims = sscanf(get_param(bodyBlock, "BrickDimensions"), "[%f %f %f]");
assert(isequal(dims(:).', [0.45, 0.32, 0.45]));

transforms = find_system(subsystem, "SearchDepth", 1, ...
    "LookUnderMasks", "all", "MaskType", "Rigid Transform");
lateralOffsets = [];
for i = 1:numel(transforms)
    if string(get_param(transforms{i}, "TranslationStandardAxis")) == "+Z"
        value = str2double(get_param(transforms{i}, "TranslationStandardOffset"));
        if isfinite(value) && abs(abs(value) - 0.2) < 1e-12
            lateralOffsets(end + 1) = value; %#ok<AGROW>
        end
    end
end
assert(numel(lateralOffsets) == 2);
assert(abs(sum(lateralOffsets)) < 1e-12);
end

function data = sampleRows(values)
data = squeeze(values.Data);
if isvector(data)
    data = data(:);
elseif size(data, 1) ~= numel(values.Time) && ...
        size(data, 2) == numel(values.Time)
    data = data.';
end
end

function finishValidation(oldFolder)
if bdIsLoaded("source")
    close_system("source", 0);
end
cd(oldFolder);
end
