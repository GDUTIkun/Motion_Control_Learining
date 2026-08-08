function result = run_trial(params, cfg)
%RUN_TRIAL Run all QR training cases for one candidate Q/R set.

trialTimer = tic;
result = struct();
result.success = false;
result.errorMessage = "";
result.params = params;

try
    addpath(cfg.simDir);
    addpath(cfg.experimentDir);

    matlabState = configureHeadlessMatlab();
    matlabCleanup = onCleanup(@() restoreHeadlessMatlab(matlabState));

    runScriptInBase(cfg.startupScript);
    applyLqrParams(params);

    model = char(cfg.modelName);
    wasLoaded = bdIsLoaded(model);
    load_system(cfg.modelPath);
    modelCleanup = onCleanup(@() closeModelIfNeeded(model, wasLoaded));

    cases = qr_training_cases();
    if isfield(cfg, "caseLimit") && ~isempty(cfg.caseLimit)
        cases = cases(1:min(numel(cases), cfg.caseLimit));
    end
    caseResults = repmat(struct("case", [], "metrics", [], "signals", []), ...
        numel(cases), 1);

    for idx = 1:numel(cases)
        fprintf("  QR trial case %d/%d: %s\n", idx, numel(cases), cases(idx).name);
        caseResults(idx) = runOneCase(cases(idx), model, cfg);
    end

    result.cases = caseResults;
    result.summary = metricsTable(caseResults);
    result.metrics = aggregateMetrics(caseResults);
    result.success = true;
catch err
    result.success = false;
    result.errorMessage = getReport(err, "extended", "hyperlinks", "off");
    result.errorIdentifier = err.identifier;
end

result.elapsedTime = toc(trialTimer);
end

function runScriptInBase(scriptPath)
assignin("base", "calibrationScriptPath", scriptPath);
evalin("base", "run(calibrationScriptPath); clear calibrationScriptPath");
end

function applyLqrParams(params)
base = evalin("base", "base");
base.Q = params.Q;
base.R = params.R;
baseLqr = floating_base_lqr_design(base);
base.command = @(x) floating_base_lqr_command(x, baseLqr);

assignin("base", "base", base);
assignin("base", "baseLqr", baseLqr);
end

function caseResult = runOneCase(caseDef, model, cfg)
setInitialState(caseDef.x0);
assignPulseVariables(caseDef);

evalin("base", "configure_discrete_controller_timing(false)");
configureHeadlessModel(model, cfg);
configurePulseGenerator(model);
set_param(model, "StopTime", char(string(caseDef.stopTime)));

simOut = sim(model, "ReturnWorkspaceOutputs", "on");

signals = extractSignalsFromLogsout(simOut.logsout);
signals = addReferences(signals);
metrics = computeMetrics(signals, caseDef);

caseResult = struct();
caseResult.case = caseDef;
caseResult.metrics = metrics;
caseResult.signals = [];
if isfield(cfg, "saveSignals") && cfg.saveSignals
caseResult.signals = signals;
end
end

function assignPulseVariables(caseDef)
assignin("base", "disturbancePulseAmplitude", caseDef.pulseAmplitudeN);
assignin("base", "disturbancePulsePeriod", caseDef.pulsePeriod);
assignin("base", "disturbancePulseWidth", caseDef.pulseWidthPercent);
assignin("base", "disturbancePulseDelay", caseDef.pulseDelay);
end

function setInitialState(x0)
assignin("base", "trialInitialState", x0(:));
evalin("base", "set_initial_base_state(trialInitialState); clear trialInitialState");
end

function matlabState = configureHeadlessMatlab()
matlabState = struct("defaultFigureVisible", []);

try
    matlabState.defaultFigureVisible = get(0, "DefaultFigureVisible");
    set(0, "DefaultFigureVisible", "off");
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

function configureHeadlessModel(model, cfg)
set_param(model, ...
    "ReturnWorkspaceOutputs", "on", ...
    "SignalLogging", "on", ...
    "SaveOutput", "off", ...
    "SaveState", "off", ...
    "SaveFinalState", "off", ...
    "SaveTime", "off", ...
    "DSMLogging", "off", ...
    "LimitDataPoints", "on", ...
    "MaxDataPoints", char(string(cfg.maxLoggedPoints)), ...
    "Decimation", char(string(cfg.scopeDecimation)));

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

for idx = 1:numel(blocks)
    set_param(blocks{idx}, ...
        "Amplitude", "disturbancePulseAmplitude", ...
        "Period", "disturbancePulsePeriod", ...
        "PulseWidth", "disturbancePulseWidth", ...
        "PhaseDelay", "disturbancePulseDelay");
end
end

function closeModelIfNeeded(model, wasLoaded)
try
    if bdIsLoaded(model)
        set_param(model, "FastRestart", "off");
    end
catch
end

if ~wasLoaded && bdIsLoaded(model)
    close_system(model, 0);
end
end

function setOptionalParam(block, paramName, value)
try
    set_param(block, paramName, value);
catch
end
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

error("run_trial:MissingLoggedSignal", ...
    "Could not find %d-wide logsout signal from %s.", width, blockPath);
end

function signals = addReferences(signals)
n = numel(signals.time);
qRef = zeros(n, 3);
dqRef = zeros(n, 3);
theta = signals.X(:, 3);
dtheta = signals.X(:, 6);

for idx = 1:n
    [qdAbs, dqdAbs] = floating_base_leg_reference( ...
        signals.time(idx), signals.X(idx, :).', [], [], [], [], ...
        signals.qpInput.data(idx, 14:15).');
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
    metrics.pulseMaxAbsTheta = windowMaxAbs(signals.base.time, theta, ...
        caseDef.pulseWindow);
    metrics.pulseMaxAbsTau = windowMaxAbs(signals.tau.time, tau, ...
        caseDef.pulseWindow);
    metrics.pulseMaxAbsULqr = windowMaxAbs(signals.uLqr.time, uLqr, ...
        caseDef.pulseWindow);
end

[metrics.stable, metrics.failureReason] = classifyStability(metrics);
end

function value = windowMaxAbs(t, data, window)
inWindow = t >= window(1) & t <= window(2);
if any(inWindow)
    value = max(abs(data(inWindow, :)), [], "all");
else
    value = NaN;
end
end

function [stable, reason] = classifyStability(metrics)
limits = struct();
limits.maxTheta = deg2rad(15);
limits.finalTheta = deg2rad(2);
limits.finalDtheta = 0.1;
limits.tauSaturation = 0.95;
limits.maxAbsX = 0.5;

reasons = {};
if metrics.maxAbsTheta > limits.maxTheta
    reasons{end + 1} = "max theta too large";
end
if abs(metrics.finalTheta) > limits.finalTheta
    reasons{end + 1} = "final theta not settled";
end
if abs(metrics.finalDtheta) > limits.finalDtheta
    reasons{end + 1} = "final dtheta not settled";
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
weight = zeros(n, 1);
initialPitchDeg = zeros(n, 1);
pulseAmplitudeN = zeros(n, 1);
stable = false(n, 1);
failureReason = strings(n, 1);
thetaRmsDeg = zeros(n, 1);
maxAbsThetaDeg = zeros(n, 1);
finalThetaDeg = zeros(n, 1);
finalX = zeros(n, 1);
maxAbsX = zeros(n, 1);
tauSaturationRatio = zeros(n, 1);
uLqrSaturationRatio = zeros(n, 1);
legPositionRms = zeros(n, 1);
legVelocityRms = zeros(n, 1);

for idx = 1:n
    c = caseResults(idx).case;
    m = caseResults(idx).metrics;
    names(idx) = c.name;
    weight(idx) = c.weight;
    initialPitchDeg(idx) = c.initialPitchDeg;
    pulseAmplitudeN(idx) = c.pulseAmplitudeN;
    stable(idx) = m.stable;
    failureReason(idx) = m.failureReason;
    thetaRmsDeg(idx) = rad2deg(m.thetaRms);
    maxAbsThetaDeg(idx) = rad2deg(m.maxAbsTheta);
    finalThetaDeg(idx) = rad2deg(m.finalTheta);
    finalX(idx) = m.finalX;
    maxAbsX(idx) = m.maxAbsX;
    tauSaturationRatio(idx) = m.tauSaturationRatio;
    uLqrSaturationRatio(idx) = m.uLqrSaturationRatio;
    legPositionRms(idx) = m.legPositionRms;
    legVelocityRms(idx) = m.legVelocityRms;
end

summary = table(names, weight, initialPitchDeg, pulseAmplitudeN, stable, ...
    failureReason, thetaRmsDeg, maxAbsThetaDeg, finalThetaDeg, finalX, ...
    maxAbsX, tauSaturationRatio, uLqrSaturationRatio, ...
    legPositionRms, legVelocityRms);
end

function metrics = aggregateMetrics(caseResults)
n = numel(caseResults);
weights = zeros(n, 1);
stable = false(n, 1);
scoreParts = zeros(n, 1);

for idx = 1:n
    weights(idx) = caseResults(idx).case.weight;
    stable(idx) = caseResults(idx).metrics.stable;
    scoreParts(idx) = NaN;
end

metrics = struct();
metrics.caseCount = n;
metrics.stableCount = sum(stable);
metrics.unstableCount = n - metrics.stableCount;
metrics.weightSum = sum(weights);
end

function path = loggedBlockPath(element)
try
    pathCell = element.BlockPath.convertToCell;
    path = string(pathCell{1});
catch
    path = string(element.BlockPath);
end
end
