function out = run_velocity_round_trip()
%RUN_VELOCITY_ROUND_TRIP Run and save the 0.5 m/s forward/reverse case.

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

model = "source";
simOut = sim(model, "StopTime", "10", "ReturnWorkspaceOutputs", "on");

metadata = captureMetadata(simDir);
rawFile = fullfile(resultDir, "raw_simulation.mat");
save(rawFile, "simOut", "metadata", "-v7.3");

signals = extractSignalsFromLogsout(simOut.logsout);
reference = buildReference(signals.time, evalin("base", "baseLqr"));

runData = struct();
runData.metadata = metadata;
runData.signals = signals;
runData.reference = reference;
runData.columns = struct( ...
    "X", ["x", "z", "theta", "dx", "dz", "dtheta"], ...
    "qRel", ["qh", "qk", "qw"], ...
    "dqRel", ["dqh", "dqk", "dqw"], ...
    "uLqr", ["FHx_ext", "FHz_ext", "MBy_des"], ...
    "tau", ["tau_h", "tau_k", "tau_w"]);

dataFile = fullfile(resultDir, "tracking_data.mat");
save(dataFile, "runData", "-v7.3");

out = struct("resultDir", string(resultDir), ...
    "rawFile", string(rawFile), "dataFile", string(dataFile));
fprintf("Saved velocity tracking data to:\n%s\n", resultDir);
end

function metadata = captureMetadata(simDir)
base = evalin("base", "base");
ctrl = evalin("base", "ctrl");
leg = evalin("base", "leg");

metadata = struct();
metadata.createdAt = datetime("now", "Format", "yyyy-MM-dd HH:mm:ss.SSS");
metadata.caseName = "velocity_round_trip_0p5mps";
metadata.modelPath = string(fullfile(simDir, "source.slx"));
metadata.stopTime = 10;
metadata.trajectory = base.trajectory;
metadata.base = struct("Q", base.Q, "R", base.R, ...
    "forceMax", base.forceMax, "momentMax", base.momentMax, ...
    "mass", base.m, "Iyy", base.Iyy);
metadata.ctrl = struct("Ts", ctrl.Ts, "qpSolver", ctrl.qpSolver, ...
    "tauMax", ctrl.tauMax, "Kp", ctrl.Kp, "Kd", ctrl.Kd);
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

signals.time = signals.base.time;
signals.X = signals.base.data(:, 2:7);
signals.qRel = signals.qpInput.data(:, 8:10);
signals.dqRel = signals.qpInput.data(:, 11:13);
end

function reference = buildReference(time, baseLqr)
n = numel(time);
XRef = zeros(n, 6);
ARef = zeros(n, 3);
for idx = 1:n
    [xRef, aRef] = floating_base_reference(time(idx), baseLqr);
    XRef(idx, :) = xRef.';
    ARef(idx, :) = aRef.';
end
reference = struct("time", time(:), "X", XRef, "acceleration", ARef);
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

error("run_velocity_round_trip:MissingLoggedSignal", ...
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

function resultDir = makeResultDir(calibrationDir)
stamp = char(datetime("now", "Format", "yyyyMMdd_HHmmss_SSS"));
resultDir = fullfile(calibrationDir, "results", "studies", ...
    "2026_08_base_motion_tracking", stamp);
mkdir(resultDir);
end

function runScriptInBase(scriptPath)
assignin("base", "studyScriptPath", scriptPath);
evalin("base", "run(studyScriptPath); clear studyScriptPath");
end
