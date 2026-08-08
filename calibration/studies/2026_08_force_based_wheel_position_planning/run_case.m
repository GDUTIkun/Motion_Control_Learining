function out = run_case(showVisual, caseMode)
%RUN_CASE Run and record the governed force-based wheel-position case.

if nargin < 1 || isempty(showVisual)
    showVisual = false;
end
if nargin < 2 || isempty(caseMode)
    caseMode = "velocity_round_trip";
end

studyDir = fileparts(mfilename("fullpath"));
calibrationDir = char(java.io.File(fullfile(studyDir, "..", "..")).getCanonicalPath());
repoRoot = char(java.io.File(fullfile(calibrationDir, "..")).getCanonicalPath());
simDir = fullfile(repoRoot, "dynamic", "2D", "simulate", "single_wheel_leg");
resultDir = makeResultDir(calibrationDir);

addpath(simDir);
originalDir = pwd;
cleanup = onCleanup(@() cd(originalDir));
cd(simDir);

runScriptInBase(fullfile(simDir, "startup.m"));
configure_base_tracking_case();
[stopTime, caseName] = configureCase(caseMode);
if showVisual
    enable_visual_simulation(false);
end
simOut = sim("source", "StopTime", string(stopTime), ...
    "ReturnWorkspaceOutputs", "on");

metadata = captureMetadata(simDir, showVisual, stopTime, caseName);
rawFile = fullfile(resultDir, "raw_simulation.mat");
save(rawFile, "simOut", "metadata", "-v7.3");

signals = extractSignalsFromLogsout(simOut.logsout);
controllerSamples = selectControllerSamples(signals.qpInput);
baseReference = buildBaseReference(controllerSamples.time, ...
    evalin("base", "baseLqr"));
wheelReference = rebuildWheelReference(controllerSamples.data);
metrics = computeMetrics(controllerSamples, signals, wheelReference, ...
    evalin("base", "ctrl"));

runData = struct();
runData.metadata = metadata;
runData.signals = signals;
runData.controllerSamples = controllerSamples;
runData.reference = struct("base", baseReference, "wheel", wheelReference);
runData.metrics = metrics;
runData.columns = struct( ...
    "X", ["x", "z", "theta", "dx", "dz", "dtheta"], ...
    "qRel", ["qh", "qk", "qw"], ...
    "dqRel", ["dqh", "dqk", "dqw"], ...
    "uLqr", ["FHx_ext", "FHz_ext", "MBy_des"], ...
    "tau", ["tau_h", "tau_k", "tau_w"]);

dataFile = fullfile(resultDir, "tracking_data.mat");
save(dataFile, "runData", "-v7.3");
summaryFile = fullfile(resultDir, "summary.txt");
writeSummary(summaryFile, metadata, metrics);

out = struct("resultDir", string(resultDir), "rawFile", string(rawFile), ...
    "dataFile", string(dataFile), "summaryFile", string(summaryFile), ...
    "metrics", metrics);
fprintf("Saved force-based wheel-position data to:\n%s\n", resultDir);
end

function metadata = captureMetadata(simDir, showVisual, stopTime, caseName)
base = evalin("base", "base");
ctrl = evalin("base", "ctrl");
leg = evalin("base", "leg");
traj = evalin("base", "traj");

metadata = struct();
metadata.createdAt = datetime("now", "Format", "yyyy-MM-dd HH:mm:ss.SSS");
metadata.caseName = string(caseName);
metadata.modelPath = string(fullfile(simDir, "source.slx"));
metadata.stopTime = stopTime;
metadata.showVisual = logical(showVisual);
metadata.trajectory = base.trajectory;
metadata.initialPose = struct( ...
    "defaultHeightReduction", traj.defaultHeightReduction, ...
    "q0", leg.q0, "wheelOffset", [traj.xO0; traj.zO0]);
metadata.wheelPlanner = struct( ...
    "enabled", traj.wheelPositionPlanning, ...
    "forceSource", traj.wheelPositionForceSource, ...
    "forceScale", traj.wheelPositionForceScale, ...
    "kneeMin", traj.wheelPositionKneeMin, ...
    "frequencyHz", traj.wheelPositionFrequencyHz, ...
    "damping", traj.wheelPositionDamping, ...
    "velocityMax", traj.wheelPositionVelocityMax, ...
    "accelerationMax", traj.wheelPositionAccelerationMax);
metadata.base = struct("Q", base.Q, "R", base.R, ...
    "forceMax", base.forceMax, "momentMax", base.momentMax, ...
    "mass", base.m, "Iyy", base.Iyy);
metadata.ctrl = struct("Ts", ctrl.Ts, "qpSolver", ctrl.qpSolver, ...
    "tauMax", ctrl.tauMax, "Kp", ctrl.Kp, "Kd", ctrl.Kd, ...
    "kneeGuardEnabled", ctrl.kneeGuardEnabled, ...
    "kneeGuardMin", ctrl.kneeGuardMin, ...
    "kneeGuardFrequencyHz", ctrl.kneeGuardFrequencyHz, ...
    "kneeGuardDamping", ctrl.kneeGuardDamping);
metadata.leg = struct("L1", leg.L1, "L2", leg.L2, "r", leg.r, ...
    "m1", leg.m1, "m2", leg.m2, "mw", leg.mw);
end

function signals = extractSignalsFromLogsout(logs)
signals = struct();
signals.base = extractLoggedMatrix(logs, "source/PD_only/Mux", 7);
signals.qpInput = extractLoggedMatrix(logs, "source/PD_only/Mux1", 16);
signals.uLqr = extractLoggedMatrix(logs, ...
    "source/PD_only/Interpreted MATLAB Function", 3);
signals.tau = extractLoggedMatrix(logs, ...
    "source/Interpreted MATLAB Function", 3);
end

function samples = selectControllerSamples(qpInput)
t = qpInput.data(:, 1);
indices = [find(diff(t) > 0); numel(t)];
samples = struct("time", t(indices), "data", qpInput.data(indices, :));
end

function reference = buildBaseReference(time, baseLqr)
n = numel(time);
X = zeros(n, 6);
acceleration = zeros(n, 3);
for idx = 1:n
    [xRef, aRef] = floating_base_reference(time(idx), baseLqr);
    X(idx, :) = xRef.';
    acceleration(idx, :) = aRef.';
end
reference = struct("time", time(:), "X", X, ...
    "acceleration", acceleration);
end

function reference = rebuildWheelReference(qpInput)
base = evalin("base", "base");
ctrl = evalin("base", "ctrl");
leg = evalin("base", "leg");
traj = evalin("base", "traj");
n = size(qpInput, 1);

reference = struct();
reference.time = qpInput(:, 1);
reference.rActual = zeros(n, 1);
reference.rEquilibrium = zeros(n, 1);
reference.rDes = zeros(n, 1);
reference.drDes = zeros(n, 1);
reference.ddrDes = zeros(n, 1);
reference.rLower = zeros(n, 1);
reference.rUpper = zeros(n, 1);
reference.force0 = zeros(n, 1);
reference.forcePlanningX = zeros(n, 1);
reference.forceTotalX = zeros(n, 1);
reference.geometryFeasible = false(n, 1);
reference.qAbs = zeros(n, 3);
reference.dqAbs = zeros(n, 3);
reference.qdAbs = zeros(n, 3);
reference.dqdAbs = zeros(n, 3);
reference.ddqdAbs = zeros(n, 3);

clear floating_base_leg_reference
for idx = 1:n
    row = qpInput(idx, :).';
    baseState = row(2:7);
    qAbs = [row(8) + baseState(3); row(9:10)];
    dqAbs = [row(11) + baseState(6); row(12:13)];
    aH = floatingHipAcceleration(baseState, row(14:15), row(16), ...
        base, ctrl);
    [qd, dqd, ddqd, plan] = floating_base_leg_reference(row(1), ...
        baseState, traj, leg, base, aH, row(14:15), true);
    kin = wheel_leg_kinematics(qAbs, dqAbs, [], leg);
    rH = rotatePitch2D(base.rHBody(:), baseState(3));

    reference.rActual(idx) = rH(1) + kin.pO(1);
    reference.rEquilibrium(idx) = plan.rXEquilibrium;
    reference.rDes(idx) = plan.rXDes;
    reference.drDes(idx) = plan.drXDes;
    reference.ddrDes(idx) = plan.ddrXDes;
    reference.rLower(idx) = plan.rXLower;
    reference.rUpper(idx) = plan.rXUpper;
    reference.force0(idx) = plan.force0;
    reference.forcePlanningX(idx) = plan.forcePlanningX;
    reference.forceTotalX(idx) = plan.FBody(1);
    reference.geometryFeasible(idx) = plan.geometryFeasible;
    reference.qAbs(idx, :) = qAbs.';
    reference.dqAbs(idx, :) = dqAbs.';
    reference.qdAbs(idx, :) = qd.';
    reference.dqdAbs(idx, :) = dqd.';
    reference.ddqdAbs(idx, :) = ddqd.';
end
end

function aH = floatingHipAcceleration(baseState, FH_ext, MBy, base, ctrl)
theta = baseState(3);
dtheta = baseState(6);
rH = rotatePitch2D(base.rHBody(:), theta);
drdtheta = [-rH(2); rH(1)];
d2rdtheta2 = -rH;
FBody = -FH_ext(:);
ddtheta = (rH(1)*FBody(2) - rH(2)*FBody(1) + MBy) / base.Iyy;
aB = [FBody(1)/base.m; FBody(2)/base.m - base.g];
aH = aB + ddtheta * drdtheta + dtheta^2 * d2rdtheta2;
if ~ctrl.useFloatingHipAcceleration
    aH = zeros(2, 1);
end
end

function metrics = computeMetrics(samples, signals, reference, ctrl)
t = samples.time;
theta = samples.data(:, 4);
tau = interp1(signals.tau.time, signals.tau.data, t, ...
    "previous", "extrap");
qError = reference.qAbs - reference.qdAbs;
rError = reference.rActual - reference.rDes;

metrics = struct();
metrics.thetaFinal = theta(end);
metrics.thetaMaxAbs = max(abs(theta));
metrics.qTrackingRms = rms(qError);
metrics.kneeActualMin = min(reference.qAbs(:, 2));
metrics.kneeReferenceMin = min(reference.qdAbs(:, 2));
metrics.wheelPositionErrorRms = rms(rError);
metrics.wheelPositionErrorMaxAbs = max(abs(rError));
metrics.wheelReferenceMaxStep = max(abs(diff(reference.rDes)));
metrics.wheelReferenceVelocityMaxAbs = max(abs(reference.drDes));
metrics.wheelReferenceAccelerationMaxAbs = max(abs(reference.ddrDes));
metrics.geometryInfeasibleRatio = mean(~reference.geometryFeasible);
metrics.equilibriumAtBoundaryRatio = mean( ...
    min(abs(reference.rEquilibrium - reference.rLower), ...
    abs(reference.rUpper - reference.rEquilibrium)) < 1e-4);
metrics.tauSaturationRatio = mean(abs(tau) >= ...
    (ctrl.tauMax(:).' - 1e-8), 1);
end

function writeSummary(filePath, metadata, metrics)
fid = fopen(filePath, "w");
if fid < 0
    error("run_case:CannotWriteSummary", ...
        "Could not write summary file: %s", filePath);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, "Force-based wheel-position planning run\n");
fprintf(fid, "=======================================\n\n");
fprintf(fid, "Created: %s\n", string(metadata.createdAt));
fprintf(fid, "Case: %s\n", metadata.caseName);
fprintf(fid, "Default height reduction: %.3f m; initial q: %s rad\n", ...
    metadata.initialPose.defaultHeightReduction, ...
    mat2str(metadata.initialPose.q0(:).', 7));
fprintf(fid, "Crouch: %.3f m, down %.3f s, recover at %.3f s over %.3f s\n", ...
    metadata.trajectory.crouchDepth, ...
    metadata.trajectory.crouchDownDuration, ...
    metadata.trajectory.crouchRecoverStart, ...
    metadata.trajectory.crouchRecoverDuration);
fprintf(fid, "Planner: enabled %d, source %s, scale %.3f\n", ...
    metadata.wheelPlanner.enabled, metadata.wheelPlanner.forceSource, ...
    metadata.wheelPlanner.forceScale);
fprintf(fid, "Governor: %.3f Hz, zeta %.3f, vMax %.3f m/s, aMax %.3f m/s^2\n", ...
    metadata.wheelPlanner.frequencyHz, metadata.wheelPlanner.damping, ...
    metadata.wheelPlanner.velocityMax, ...
    metadata.wheelPlanner.accelerationMax);
fprintf(fid, "QP solver: %s\n\n", metadata.ctrl.qpSolver);
fprintf(fid, "theta final/maxabs: %.9g / %.9g rad\n", ...
    metrics.thetaFinal, metrics.thetaMaxAbs);
fprintf(fid, "q tracking RMS: %s rad\n", mat2str(metrics.qTrackingRms, 7));
fprintf(fid, "knee actual/reference min: %.9g / %.9g rad\n", ...
    metrics.kneeActualMin, metrics.kneeReferenceMin);
fprintf(fid, "wheel position error RMS/maxabs: %.9g / %.9g m\n", ...
    metrics.wheelPositionErrorRms, metrics.wheelPositionErrorMaxAbs);
fprintf(fid, "wheel reference max step: %.9g m\n", ...
    metrics.wheelReferenceMaxStep);
fprintf(fid, "wheel reference speed/accel maxabs: %.9g / %.9g\n", ...
    metrics.wheelReferenceVelocityMaxAbs, ...
    metrics.wheelReferenceAccelerationMaxAbs);
fprintf(fid, "geometry infeasible ratio: %.9g\n", ...
    metrics.geometryInfeasibleRatio);
fprintf(fid, "equilibrium at boundary ratio: %.9g\n", ...
    metrics.equilibriumAtBoundaryRatio);
fprintf(fid, "torque saturation ratio: %s\n", ...
    mat2str(metrics.tauSaturationRatio, 7));
end

function sig = extractLoggedMatrix(logs, blockPath, width)
for idx = 1:logs.numElements
    element = logs.get(idx);
    if loggedBlockPath(element) ~= string(blockPath)
        continue;
    end

    values = element.Values;
    data = squeeze(double(values.Data));
    if isvector(data)
        data = data(:);
    elseif size(data, 1) ~= numel(values.Time) ...
            && size(data, 2) == numel(values.Time)
        data = data.';
    end

    if size(data, 2) == width
        sig = struct("time", values.Time(:), "data", data);
        return;
    end
end

error("run_case:MissingLoggedSignal", ...
    "Could not find %d-wide logsout signal from %s.", width, blockPath);
end

function path = loggedBlockPath(element)
try
    pathCell = element.BlockPath.convertToCell;
    path = string(pathCell{1});
catch
    path = string(element.BlockPath);
end
end

function rWorld = rotatePitch2D(rBody, theta)
rWorld = [
    cos(theta)*rBody(1) - sin(theta)*rBody(2);
    sin(theta)*rBody(1) + cos(theta)*rBody(2)
];
end

function resultDir = makeResultDir(calibrationDir)
stamp = char(datetime("now", "Format", "yyyyMMdd_HHmmss_SSS"));
resultDir = fullfile(calibrationDir, "results", "studies", ...
    "2026_08_force_based_wheel_position_planning", stamp);
mkdir(resultDir);
end

function [stopTime, caseName] = configureCase(caseMode)
caseMode = lower(string(caseMode));
switch caseMode
    case "standing"
        evalin("base", ...
            "base.trajectory.enabled=false; baseLqr.trajectory.enabled=false;");
        stopTime = 5;
        caseName = "force_based_wheel_position_standing";
    case "velocity_round_trip"
        stopTime = 10;
        caseName = "force_based_wheel_position_velocity_round_trip";
    case "velocity_round_trip_crouch"
        evalin("base", ...
            "base.trajectory.crouchDepth=0.025; baseLqr.trajectory.crouchDepth=0.025;");
        stopTime = 10;
        caseName = "force_based_wheel_position_velocity_round_trip_crouch";
    case "velocity_round_trip_no_planner"
        evalin("base", "traj.wheelPositionPlanning=false;");
        stopTime = 10;
        caseName = "velocity_round_trip_no_wheel_position_planner";
    otherwise
        error("run_case:InvalidCase", ...
            ["caseMode must be 'standing', 'velocity_round_trip', " ...
            "'velocity_round_trip_crouch', or " ...
            "'velocity_round_trip_no_planner'."]);
end
end

function runScriptInBase(scriptPath)
assignin("base", "studyScriptPath", scriptPath);
evalin("base", "run(studyScriptPath); clear studyScriptPath");
end
