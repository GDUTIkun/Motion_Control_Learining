function out = run_cases(cases)
%RUN_CASES Run the LQR + QP disturbance-response study cases.

if nargin < 1 || isempty(cases)
    cases = disturbance_cases();
end

studyDir = fileparts(mfilename("fullpath"));
calibrationDir = char(java.io.File(fullfile(studyDir, "..", "..")).getCanonicalPath());
repoRoot = char(java.io.File(fullfile(calibrationDir, "..")).getCanonicalPath());
simDir = fullfile(repoRoot, "dynamic", "2D", "simulate", "single_wheel_leg");
resultDir = makeResultDir(calibrationDir);

addpath(simDir);
addpath(studyDir);

out = struct();
out.studyDir = studyDir;
out.calibrationDir = calibrationDir;
out.simDir = simDir;
out.resultDir = resultDir;
out.cases = repmat(struct("case", [], "signals", [], "metrics", []), numel(cases), 1);

for idx = 1:numel(cases)
    fprintf("\nRunning case %d/%d: %s\n", idx, numel(cases), cases(idx).name);
    caseResult = runOneCase(cases(idx), simDir);
    out.cases(idx) = caseResult;
    save(fullfile(resultDir, sprintf("%s.mat", safeFileStem(cases(idx).name))), ...
        "caseResult");
end

summary = metricsTable(out.cases);
out.summary = summary;
save(fullfile(resultDir, "study_results.mat"), "out", "summary");
writetable(summary, fullfile(resultDir, "summary.csv"));

fprintf("\nSaved study results to:\n%s\n", resultDir);
disp(summary);
end

function caseResult = runOneCase(caseDef, simDir)
originalDir = pwd;
cleanup = onCleanup(@() cd(originalDir));
cd(simDir);

matlabState = configureHeadlessMatlab();
matlabCleanup = onCleanup(@() restoreHeadlessMatlab(matlabState));

runScriptInBase(fullfile(simDir, "startup.m"));
setInitialStateInBase(caseDef.x0);
assignPulseVariables(caseDef);

model = "source";
wasLoaded = bdIsLoaded(model);
evalin("base", "configure_model(false)");
load_system(model);
modelCleanup = onCleanup(@() closeModelIfNeeded(model, wasLoaded));
set_param(model, "StopTime", char(string(caseDef.stopTime)));
configureHeadlessModel(model);
configurePulseGenerator(model);

Simulink.sdi.clear;
Simulink.sdi.setRecordData(true);
markSignal("source/PD_only/Mux");                         % base state input
markSignal("source/PD_only/Mux1");                        % QP input
markSignal("source/PD_only/Interpreted MATLAB Function"); % LQR command
markSignal("source/Interpreted MATLAB Function");         % tau

sim(model, "ReturnWorkspaceOutputs", "on");

signals = extractSignalsFromSdi();
signals = addReferences(signals);
metrics = computeMetrics(signals, caseDef);

caseResult = struct();
caseResult.case = caseDef;
caseResult.signals = signals;
caseResult.metrics = metrics;
end

function markSignal(block)
Simulink.sdi.markSignalForStreaming(block, 1, "on");
end

function assignPulseVariables(caseDef)
assignin("base", "disturbancePulseAmplitude", caseDef.pulseAmplitudeN);
assignin("base", "disturbancePulsePeriod", caseDef.pulsePeriod);
assignin("base", "disturbancePulseWidth", caseDef.pulseWidthPercent);
assignin("base", "disturbancePulseDelay", caseDef.pulseDelay);
end

function setInitialStateInBase(x0)
assignin("base", "studyInitialState", x0(:));
evalin("base", "set_initial_base_state(studyInitialState); clear studyInitialState");
end

function matlabState = configureHeadlessMatlab()
matlabState = struct("defaultFigureVisible", []);

try
    matlabState.defaultFigureVisible = get(0, "DefaultFigureVisible");
    set(0, "DefaultFigureVisible", "off");
catch
end

try
    Simulink.sdi.clear;
catch
end
end

function restoreHeadlessMatlab(matlabState)
if isstruct(matlabState) && isfield(matlabState, "defaultFigureVisible") ...
        && ~isempty(matlabState.defaultFigureVisible)
    try
        set(0, "DefaultFigureVisible", matlabState.defaultFigureVisible);
    catch
    end
end
end

function configureHeadlessModel(model)
set_param(model, ...
    "ReturnWorkspaceOutputs", "on", ...
    "SignalLogging", "on", ...
    "SaveOutput", "off", ...
    "SaveState", "off", ...
    "SaveFinalState", "off", ...
    "SaveTime", "off", ...
    "DSMLogging", "off", ...
    "LimitDataPoints", "on", ...
    "MaxDataPoints", "20000", ...
    "Decimation", "1");

setOptionalParam(model, "SimscapeLogType", "none");
setOptionalParam(model, "SimMechanicsOpenEditorOnUpdate", "off");

scopes = find_system(model, "LookUnderMasks", "all", ...
    "FollowLinks", "on", "BlockType", "Scope");
for idx = 1:numel(scopes)
    setOptionalParam(scopes{idx}, "OpenAtSimulationStart", "off");
    setOptionalParam(scopes{idx}, "SaveToWorkspace", "off");
end
end

function configurePulseGenerator(model)
blocks = find_system(model, "LookUnderMasks", "all", ...
    "FollowLinks", "on", "BlockType", "DiscretePulseGenerator");

if isempty(blocks)
    warning("run_cases:MissingPulseGenerator", ...
        "No Pulse Generator block was found in %s.", model);
    return;
end

for idx = 1:numel(blocks)
    set_param(blocks{idx}, ...
        "Amplitude", "disturbancePulseAmplitude", ...
        "Period", "disturbancePulsePeriod", ...
        "PulseWidth", "disturbancePulseWidth", ...
        "PhaseDelay", "disturbancePulseDelay");
end
end

function closeModelIfNeeded(model, wasLoaded)
if ~wasLoaded && bdIsLoaded(model)
    close_system(model, 0);
end
end

function runScriptInBase(scriptPath)
assignin("base", "studyScriptPath", scriptPath);
evalin("base", "run(studyScriptPath); clear studyScriptPath");
end

function setOptionalParam(block, paramName, value)
try
    set_param(block, paramName, value);
catch
end
end

function signals = extractSignalsFromSdi()
runIDs = Simulink.sdi.getAllRunIDs;
if isempty(runIDs)
    error("run_cases:MissingSdiRun", "No SDI run was recorded.");
end
run = Simulink.sdi.getRun(runIDs(end));

signals = struct();
signals.base = extractVectorSignal(run, "source/PD_only/Mux", 7);
signals.qpInput = extractVectorSignal(run, "source/PD_only/Mux1", 16);
signals.uLqr = extractScalarVector(run, ...
    "source/PD_only/Interpreted MATLAB Function", 3);
signals.tau = extractScalarVector(run, ...
    "source/Interpreted MATLAB Function", 3);

signals.time = signals.base.time;
signals.X = signals.base.data(:, 2:7);
signals.qRel = signals.qpInput.data(:, 8:10);
signals.dqRel = signals.qpInput.data(:, 11:13);
end

function sig = extractVectorSignal(run, blockPath, width)
for idx = 1:run.SignalCount
    s = run.getSignalByIndex(idx);
    if blockPathOf(s) == string(blockPath)
        values = s.Values;
        data = double(values.Data);
        if size(data, 2) == width
            sig = struct("time", values.Time(:), "data", data);
            return;
        end
    end
end
error("run_cases:MissingVectorSignal", ...
    "Could not find %d-wide signal from %s.", width, blockPath);
end

function sig = extractScalarVector(run, blockPath, width)
parts = cell(width, 1);
time = [];
for idx = 1:run.SignalCount
    s = run.getSignalByIndex(idx);
    if blockPathOf(s) ~= string(blockPath)
        continue;
    end
    component = parseComponentIndex(s.Name);
    if component >= 1 && component <= width
        values = s.Values;
        parts{component} = double(values.Data(:));
        if isempty(time)
            time = values.Time(:);
        end
    end
end

if isempty(time) || any(cellfun(@isempty, parts))
    error("run_cases:MissingScalarVector", ...
        "Could not assemble %d-wide signal from %s.", width, blockPath);
end

data = zeros(numel(time), width);
for idx = 1:width
    data(:, idx) = parts{idx};
end
sig = struct("time", time, "data", data);
end

function signals = addReferences(signals)
n = numel(signals.time);
qRef = zeros(n, 3);
dqRef = zeros(n, 3);
theta = signals.X(:, 3);
dtheta = signals.X(:, 6);

for idx = 1:n
    [qdAbs, dqdAbs] = wheel_leg_reference(signals.time(idx));
    qRef(idx, :) = [qdAbs(1) - theta(idx), qdAbs(2), qdAbs(3)];
    dqRef(idx, :) = [dqdAbs(1) - dtheta(idx), dqdAbs(2), dqdAbs(3)];
end

signals.qRef = qRef;
signals.dqRef = dqRef;
signals.legQError = signals.qRel(:, 1:2) - qRef(:, 1:2);
signals.legDqError = signals.dqRel(:, 1:2) - dqRef(:, 1:2);
end

function metrics = computeMetrics(signals, caseDef)
ctrl = evalin("base", "ctrl");
base = evalin("base", "base");

theta = signals.X(:, 3);
dtheta = signals.X(:, 6);
x = signals.X(:, 1);
dx = signals.X(:, 4);
tau = signals.tau.data;
uLqr = signals.uLqr.data;

thetaBand = deg2rad(0.5);
metrics = struct();
metrics.maxAbsTheta = max(abs(theta));
metrics.thetaRms = rms(theta);
metrics.finalTheta = theta(end);
metrics.dthetaRms = rms(dtheta);
metrics.finalDtheta = dtheta(end);
metrics.finalX = x(end);
metrics.finalDx = dx(end);
metrics.maxAbsX = max(abs(x));
metrics.settlingTimeTheta = settlingTime(signals.time, theta, thetaBand);
metrics.maxAbsTauH = max(abs(tau(:, 1)));
metrics.maxAbsTauK = max(abs(tau(:, 2)));
metrics.maxAbsTauW = max(abs(tau(:, 3)));
metrics.tauSaturationRatio = max(abs(tau) ./ ctrl.tauMax(:).', [], "all");
metrics.uLqrSaturationRatio = max(abs(uLqr) ./ ...
    [base.forceMax(:).', base.momentMax], [], "all");
metrics.legPositionRms = rms(signals.legQError, "all");
metrics.legVelocityRms = rms(signals.legDqError, "all");
metrics.pulseMaxAbsTheta = NaN;
metrics.pulseMaxAbsTau = NaN;
metrics.pulseMaxAbsULqr = NaN;

if isfield(caseDef, "pulseWindow") && all(isfinite(caseDef.pulseWindow))
    inPulse = signals.time >= caseDef.pulseWindow(1) ...
        & signals.time <= caseDef.pulseWindow(2);
    if any(inPulse)
        metrics.pulseMaxAbsTheta = max(abs(theta(inPulse)));
        metrics.pulseMaxAbsTau = max(abs(tau(inPulse, :)), [], "all");
        metrics.pulseMaxAbsULqr = max(abs(uLqr(inPulse, :)), [], "all");
    end
end

[metrics.stable, metrics.failureReason] = classifyStability(metrics);
end

function [stable, reason] = classifyStability(metrics)
limits = struct();
limits.maxTheta = deg2rad(10);
limits.finalTheta = deg2rad(1);
limits.tauSaturation = 0.95;
limits.maxAbsX = 0.5;

reasons = {};
if metrics.maxAbsTheta > limits.maxTheta
    reasons{end + 1} = "max theta too large";
end
if abs(metrics.finalTheta) > limits.finalTheta
    reasons{end + 1} = "final theta not settled";
end
if metrics.tauSaturationRatio > limits.tauSaturation
    reasons{end + 1} = "tau near saturation";
end
if metrics.maxAbsX > limits.maxAbsX
    reasons{end + 1} = "x drift too large";
end

stable = isempty(reasons);
if stable
    reason = "ok";
else
    reason = strjoin(string(reasons), "; ");
end
end

function tSettle = settlingTime(t, y, band)
outside = find(abs(y) > band);
if isempty(outside)
    tSettle = t(1);
elseif outside(end) == numel(t)
    tSettle = NaN;
else
    tSettle = t(outside(end) + 1);
end
end

function summary = metricsTable(caseResults)
n = numel(caseResults);
names = strings(n, 1);
initialPitchDeg = zeros(n, 1);
pulseAmplitudeN = zeros(n, 1);
maxAbsThetaDeg = zeros(n, 1);
finalThetaDeg = zeros(n, 1);
finalX = zeros(n, 1);
settlingTimeTheta = zeros(n, 1);
tauSaturationRatio = zeros(n, 1);
legPositionRms = zeros(n, 1);
legVelocityRms = zeros(n, 1);
pulseMaxAbsThetaDeg = NaN(n, 1);
pulseMaxAbsTau = NaN(n, 1);
pulseMaxAbsULqr = NaN(n, 1);
stable = false(n, 1);
failureReason = strings(n, 1);

for idx = 1:n
    m = caseResults(idx).metrics;
    c = caseResults(idx).case;
    names(idx) = c.name;
    initialPitchDeg(idx) = c.initialPitchDeg;
    pulseAmplitudeN(idx) = c.pulseAmplitudeN;
    maxAbsThetaDeg(idx) = rad2deg(m.maxAbsTheta);
    finalThetaDeg(idx) = rad2deg(m.finalTheta);
    finalX(idx) = m.finalX;
    settlingTimeTheta(idx) = m.settlingTimeTheta;
    tauSaturationRatio(idx) = m.tauSaturationRatio;
    legPositionRms(idx) = m.legPositionRms;
    legVelocityRms(idx) = m.legVelocityRms;
    pulseMaxAbsThetaDeg(idx) = rad2deg(m.pulseMaxAbsTheta);
    pulseMaxAbsTau(idx) = m.pulseMaxAbsTau;
    pulseMaxAbsULqr(idx) = m.pulseMaxAbsULqr;
    stable(idx) = m.stable;
    failureReason(idx) = m.failureReason;
end

summary = table(names, initialPitchDeg, pulseAmplitudeN, stable, ...
    failureReason, maxAbsThetaDeg, finalThetaDeg, finalX, ...
    settlingTimeTheta, tauSaturationRatio, legPositionRms, legVelocityRms, ...
    pulseMaxAbsThetaDeg, pulseMaxAbsTau, pulseMaxAbsULqr);
end

function resultDir = makeResultDir(calibrationDir)
stamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
resultDir = fullfile(calibrationDir, "results", "studies", ...
    "2026_08_lqr_disturbance_response", char(stamp));
if ~isfolder(resultDir)
    mkdir(resultDir);
end
end

function stem = safeFileStem(name)
stem = regexprep(char(string(name)), "[^a-zA-Z0-9_\\-]", "_");
end

function path = blockPathOf(signal)
pathObj = signal.BlockPath;
if isa(pathObj, "Simulink.SimulationData.BlockPath")
    pathCell = pathObj.convertToCell;
    path = string(pathCell{1});
else
    path = string(pathObj);
end
end

function component = parseComponentIndex(name)
tokens = regexp(char(name), "\((\d+)\)$", "tokens", "once");
if isempty(tokens)
    component = NaN;
else
    component = str2double(tokens{1});
end
end
