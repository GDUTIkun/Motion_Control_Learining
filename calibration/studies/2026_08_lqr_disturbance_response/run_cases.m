function out = run_cases(cases, lqrParams, limitParams, ctrlParams)
%RUN_CASES Run the LQR + QP disturbance-response study cases.

if nargin < 1 || isempty(cases)
    cases = disturbance_cases();
end
if nargin < 2
    lqrParams = [];
end
if nargin < 3
    limitParams = [];
end
if nargin < 4
    ctrlParams = [];
end

studyDir = fileparts(mfilename("fullpath"));
calibrationDir = char(java.io.File(fullfile(studyDir, "..", "..")).getCanonicalPath());
repoRoot = char(java.io.File(fullfile(calibrationDir, "..")).getCanonicalPath());
simDir = fullfile(repoRoot, "model", "simulate", "base_with_wheel_leg");
resultDir = makeResultDir(calibrationDir);

addpath(simDir);
addpath(studyDir);

out = struct();
out.studyDir = studyDir;
out.calibrationDir = calibrationDir;
out.simDir = simDir;
out.resultDir = resultDir;
out.lqrParams = lqrParams;
out.limitParams = limitParams;
out.ctrlParams = ctrlParams;
out.cases = repmat(struct("case", [], "signals", [], "metrics", []), numel(cases), 1);

for idx = 1:numel(cases)
    fprintf("\nRunning case %d/%d: %s\n", idx, numel(cases), cases(idx).name);
    try
        caseResult = runOneCase(cases(idx), simDir, lqrParams, limitParams, ctrlParams);
    catch err
        warning("run_cases:CaseSimulationFailed", ...
            "Case %s failed and will be recorded as unstable.\n%s", ...
            cases(idx).name, err.message);
        caseResult = makeFailedCaseResult(cases(idx), err);
    end
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

function caseResult = runOneCase(caseDef, simDir, lqrParams, limitParams, ctrlParams)
caseTimer = tic;
originalDir = pwd;
cleanup = onCleanup(@() cd(originalDir));
cd(simDir);

matlabState = configureHeadlessMatlab();
matlabCleanup = onCleanup(@() restoreHeadlessMatlab(matlabState));

runScriptInBase(fullfile(simDir, "startup.m"));
applyLimitParamsIfNeeded(limitParams);
applyLqrParamsIfNeeded(lqrParams);
applyCtrlParamsIfNeeded(ctrlParams);
setInitialStateInBase(caseDef.x0);
assignPulseVariables(caseDef);

model = "source";
wasLoaded = bdIsLoaded(model);
evalin("base", "configure_discrete_controller_timing(false)");
load_system(model);
modelCleanup = onCleanup(@() closeModelIfNeeded(model, wasLoaded));
set_param(model, "StopTime", char(string(caseDef.stopTime)));
configureHeadlessModel(model);
configurePulseGenerator(model);

simOut = sim(model, "ReturnWorkspaceOutputs", "on");

signals = extractSignalsFromLogsout(simOut.logsout);
signals = addReferences(signals);
metrics = computeMetrics(signals, caseDef);
metrics.elapsedSeconds = toc(caseTimer);

caseResult = struct();
caseResult.case = caseDef;
caseResult.signals = signals;
caseResult.metrics = metrics;
end

function caseResult = makeFailedCaseResult(caseDef, err)
metrics = failedMetrics(err);
caseResult = struct();
caseResult.case = caseDef;
caseResult.signals = struct();
caseResult.metrics = metrics;
end

function metrics = failedMetrics(err)
metrics = struct();
metrics.maxAbsTheta = NaN;
metrics.thetaRms = NaN;
metrics.finalTheta = NaN;
metrics.dthetaRms = NaN;
metrics.finalDtheta = NaN;
metrics.finalX = NaN;
metrics.finalDx = NaN;
metrics.maxAbsX = NaN;
metrics.settlingTimeTheta = NaN;
metrics.maxAbsTauH = NaN;
metrics.maxAbsTauK = NaN;
metrics.maxAbsTauW = NaN;
metrics.tauSaturationRatio = NaN;
metrics.uLqrSaturationRatio = NaN;
metrics.legPositionRms = NaN;
metrics.legVelocityRms = NaN;
metrics.legBranchOk = false;
metrics.minAbsSinQkPost = NaN;
metrics.minAbsDetJPost = NaN;
metrics.minQkDegPost = NaN;
metrics.maxQkDegPost = NaN;
metrics.finalQkDeg = NaN;
metrics.maxAbsPOxPost = NaN;
metrics.finalPOx = NaN;
metrics.maxLegQErrorPost = NaN;
metrics.maxLegDqErrorPost = NaN;
metrics.pulseMaxAbsTheta = NaN;
metrics.pulseMaxAbsTau = NaN;
metrics.pulseMaxAbsULqr = NaN;
metrics.elapsedSeconds = NaN;
metrics.stable = false;
metrics.failureReason = "simulation error: " + compactErrorMessage(err.message);
metrics.simErrorIdentifier = string(err.identifier);
end

function text = compactErrorMessage(text)
text = string(text);
text = replace(text, newline, " ");
maxChars = 240;
if strlength(text) > maxChars
    text = extractBefore(text, maxChars) + "...";
end
end

function applyCtrlParamsIfNeeded(ctrlParams)
if isempty(ctrlParams)
    return;
end

ctrl = evalin("base", "ctrl");

if isfield(ctrlParams, "KpScale")
    scale = normalizeScale(ctrlParams.KpScale, 3, "KpScale");
    ctrl.Kp = diag(scale) * ctrl.Kp;
end
if isfield(ctrlParams, "KdScale")
    scale = normalizeScale(ctrlParams.KdScale, 3, "KdScale");
    ctrl.Kd = diag(scale) * ctrl.Kd;
end
if isfield(ctrlParams, "qpWqddScale")
    scale = normalizeScale(ctrlParams.qpWqddScale, 3, "qpWqddScale");
    ctrl.qpWqdd = ctrl.qpWqdd(:) .* scale;
end
if isfield(ctrlParams, "qpWtauScale")
    scale = normalizeScale(ctrlParams.qpWtauScale, 3, "qpWtauScale");
    ctrl.qpWtau = ctrl.qpWtau(:) .* scale;
end
if isfield(ctrlParams, "qpWFcScale")
    scale = normalizeScale(ctrlParams.qpWFcScale, 2, "qpWFcScale");
    ctrl.qpWFc = ctrl.qpWFc(:) .* scale;
end
if isfield(ctrlParams, "constraintVelocityGainScale")
    ctrl.constraintVelocityGain = ctrl.constraintVelocityGain ...
        * double(ctrlParams.constraintVelocityGainScale);
end
if isfield(ctrlParams, "useFloatingHipAcceleration")
    ctrl.useFloatingHipAcceleration = logical(ctrlParams.useFloatingHipAcceleration);
end

assignin("base", "ctrl", ctrl);
end

function scale = normalizeScale(value, n, fieldName)
scale = double(value(:));
if isscalar(scale)
    scale = repmat(scale, n, 1);
end
if numel(scale) ~= n || any(~isfinite(scale)) || any(scale <= 0)
    error("run_cases:InvalidCtrlScale", ...
        "%s must be a positive scalar or %d-element vector.", fieldName, n);
end
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

function applyLimitParamsIfNeeded(limitParams)
if isempty(limitParams)
    return;
end

if isnumeric(limitParams) && isscalar(limitParams)
    limitParams = struct( ...
        "tauScale", double(limitParams), ...
        "forceScale", double(limitParams), ...
        "momentScale", double(limitParams));
end

tauScale = getFieldOrDefault(limitParams, "tauScale", 1.0);
forceScale = getFieldOrDefault(limitParams, "forceScale", 1.0);
momentScale = getFieldOrDefault(limitParams, "momentScale", 1.0);

if tauScale <= 0 || forceScale <= 0 || momentScale <= 0
    error("run_cases:InvalidLimitScale", ...
        "Limit scales must be positive.");
end

ctrl = evalin("base", "ctrl");
base = evalin("base", "base");
hip = evalin("base", "hip");

ctrl.tauMax = ctrl.tauMax(:) * tauScale;
hip.forceMax = hip.forceMax(:) * forceScale;
base.forceMax = base.forceMax(:) * forceScale;
base.momentMax = base.momentMax * momentScale;

baseLqr = floating_base_lqr_design(base);
base.command = @(x) floating_base_lqr_command(x, baseLqr);

assignin("base", "ctrl", ctrl);
assignin("base", "hip", hip);
assignin("base", "base", base);
assignin("base", "baseLqr", baseLqr);
end

function applyLqrParamsIfNeeded(lqrParams)
if isempty(lqrParams)
    return;
end

requiredFields = ["Q", "R"];
for idx = 1:numel(requiredFields)
    if ~isfield(lqrParams, char(requiredFields(idx)))
        error("run_cases:InvalidLqrParams", ...
            "lqrParams must contain fields Q and R.");
    end
end

assignin("base", "studyLqrParams", lqrParams);
cmd = [ ...
    'base.Q = studyLqrParams.Q;' ...
    'base.R = studyLqrParams.R;' ...
    'baseLqr = floating_base_lqr_design(base);' ...
    'base.command = @(x) floating_base_lqr_command(x, baseLqr);' ...
    'clear studyLqrParams;' ...
    ];
evalin("base", cmd);

if isfield(lqrParams, "commandShaping")
    base = evalin("base", "base");
    baseLqr = evalin("base", "baseLqr");
    base.commandShaping = lqrParams.commandShaping;
    baseLqr.commandShaping = lqrParams.commandShaping;
    base.command = @(x) floating_base_lqr_command(x, baseLqr);
    assignin("base", "base", base);
    assignin("base", "baseLqr", baseLqr);
end
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

error("run_cases:MissingLoggedSignal", ...
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
leg = evalin("base", "leg");

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
[legMetrics, legBranchOk] = computeLegGeometryMetrics(signals, caseDef, leg);
metrics.legBranchOk = legBranchOk;
metrics.minAbsSinQkPost = legMetrics.minAbsSinQkPost;
metrics.minAbsDetJPost = legMetrics.minAbsDetJPost;
metrics.minQkDegPost = legMetrics.minQkDegPost;
metrics.maxQkDegPost = legMetrics.maxQkDegPost;
metrics.finalQkDeg = legMetrics.finalQkDeg;
metrics.maxAbsPOxPost = legMetrics.maxAbsPOxPost;
metrics.finalPOx = legMetrics.finalPOx;
metrics.maxLegQErrorPost = legMetrics.maxLegQErrorPost;
metrics.maxLegDqErrorPost = legMetrics.maxLegDqErrorPost;
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

function [legMetrics, legBranchOk] = computeLegGeometryMetrics(signals, caseDef, leg)
t = signals.time(:);
theta = signals.X(:, 3);
dtheta = signals.X(:, 6);
qRel = signals.qRel;
dqRel = signals.dqRel;
qAbs = [qRel(:, 1) + theta, qRel(:, 2), qRel(:, 3)];
dqAbs = [dqRel(:, 1) + dtheta, dqRel(:, 2), dqRel(:, 3)];

n = numel(t);
pOx = zeros(n, 1);
detJO = zeros(n, 1);
for idx = 1:n
    kin = wheel_leg_kinematics(qAbs(idx, :).', dqAbs(idx, :).', ...
        zeros(3, 1), leg);
    pOx(idx) = kin.pO(1);
    detJO(idx) = det(kin.JO(:, 1:2));
end

if isfield(caseDef, "pulseWindow") && all(isfinite(caseDef.pulseWindow))
    post = t >= caseDef.pulseWindow(2);
else
    post = true(size(t));
end

qkDeg = rad2deg(qAbs(:, 2));
qErrNorm = vecnorm(signals.legQError, 2, 2);
dqErrNorm = vecnorm(signals.legDqError, 2, 2);

legMetrics = struct();
legMetrics.minAbsSinQkPost = min(abs(sin(qAbs(post, 2))));
legMetrics.minAbsDetJPost = min(abs(detJO(post)));
legMetrics.minQkDegPost = min(qkDeg(post));
legMetrics.maxQkDegPost = max(qkDeg(post));
legMetrics.finalQkDeg = qkDeg(end);
legMetrics.maxAbsPOxPost = max(abs(pOx(post)));
legMetrics.finalPOx = pOx(end);
legMetrics.maxLegQErrorPost = max(qErrNorm(post));
legMetrics.maxLegDqErrorPost = max(dqErrNorm(post));

legBranchOk = legMetrics.minAbsSinQkPost > sind(5) ...
    && legMetrics.minQkDegPost > 5 ...
    && legMetrics.maxQkDegPost < 120 ...
    && legMetrics.maxAbsPOxPost < 0.25;
end

function value = windowMaxAbs(t, data, window)
inWindow = t >= window(1) & t <= window(2);
if any(inWindow)
    value = max(abs(data(inWindow, :)), [], "all");
else
    value = NaN;
end
end

function value = getFieldOrDefault(s, name, defaultValue)
if isfield(s, name)
    value = s.(name);
else
    value = defaultValue;
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
initialPitchDeg = zeros(n, 1);
pulseAmplitudeN = zeros(n, 1);
maxAbsThetaDeg = zeros(n, 1);
finalThetaDeg = zeros(n, 1);
finalDtheta = zeros(n, 1);
finalX = zeros(n, 1);
settlingTimeTheta = zeros(n, 1);
tauSaturationRatio = zeros(n, 1);
legPositionRms = zeros(n, 1);
legVelocityRms = zeros(n, 1);
legBranchOk = false(n, 1);
minAbsSinQkPost = zeros(n, 1);
minAbsDetJPost = zeros(n, 1);
minQkDegPost = zeros(n, 1);
maxQkDegPost = zeros(n, 1);
finalQkDeg = zeros(n, 1);
maxAbsPOxPost = zeros(n, 1);
finalPOx = zeros(n, 1);
maxLegQErrorPost = zeros(n, 1);
maxLegDqErrorPost = zeros(n, 1);
pulseMaxAbsThetaDeg = NaN(n, 1);
pulseMaxAbsTau = NaN(n, 1);
pulseMaxAbsULqr = NaN(n, 1);
elapsedSeconds = zeros(n, 1);
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
    finalDtheta(idx) = m.finalDtheta;
    finalX(idx) = m.finalX;
    settlingTimeTheta(idx) = m.settlingTimeTheta;
    tauSaturationRatio(idx) = m.tauSaturationRatio;
    legPositionRms(idx) = m.legPositionRms;
    legVelocityRms(idx) = m.legVelocityRms;
    legBranchOk(idx) = m.legBranchOk;
    minAbsSinQkPost(idx) = m.minAbsSinQkPost;
    minAbsDetJPost(idx) = m.minAbsDetJPost;
    minQkDegPost(idx) = m.minQkDegPost;
    maxQkDegPost(idx) = m.maxQkDegPost;
    finalQkDeg(idx) = m.finalQkDeg;
    maxAbsPOxPost(idx) = m.maxAbsPOxPost;
    finalPOx(idx) = m.finalPOx;
    maxLegQErrorPost(idx) = m.maxLegQErrorPost;
    maxLegDqErrorPost(idx) = m.maxLegDqErrorPost;
    pulseMaxAbsThetaDeg(idx) = rad2deg(m.pulseMaxAbsTheta);
    pulseMaxAbsTau(idx) = m.pulseMaxAbsTau;
    pulseMaxAbsULqr(idx) = m.pulseMaxAbsULqr;
    elapsedSeconds(idx) = m.elapsedSeconds;
    stable(idx) = m.stable;
    failureReason(idx) = m.failureReason;
end

summary = table(names, initialPitchDeg, pulseAmplitudeN, stable, ...
    failureReason, maxAbsThetaDeg, finalThetaDeg, finalDtheta, finalX, ...
    settlingTimeTheta, tauSaturationRatio, legPositionRms, legVelocityRms, ...
    legBranchOk, minAbsSinQkPost, minAbsDetJPost, minQkDegPost, ...
    maxQkDegPost, finalQkDeg, maxAbsPOxPost, finalPOx, ...
    maxLegQErrorPost, maxLegDqErrorPost, pulseMaxAbsThetaDeg, ...
    pulseMaxAbsTau, pulseMaxAbsULqr, elapsedSeconds);
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

function path = loggedBlockPath(element)
try
    pathCell = element.BlockPath.convertToCell;
    path = string(pathCell{1});
catch
    path = string(element.BlockPath);
end
end
