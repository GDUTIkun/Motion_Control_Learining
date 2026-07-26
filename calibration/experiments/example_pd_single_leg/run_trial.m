function result = run_trial(params, cfg)
%RUN_TRIAL Run one single_leg Simulink trial for a candidate parameter set.

trialTimer = tic;
result = struct();
result.success = false;
result.errorMessage = "";
result.params = params;

try
    addpath(cfg.simDir);
    run(cfg.startupScript);

    modelName = char(cfg.modelName);
    wasLoaded = bdIsLoaded(modelName);
    load_system(cfg.modelPath);

    modelState = captureModelState(modelName, cfg);
    cleanup = onCleanup(@() restoreModelState(modelName, wasLoaded, modelState));

    applyControllerParams(params);
    configureTrialModel(modelName, cfg);

    simOut = sim(modelName, "ReturnWorkspaceOutputs", "on");
    signals = extractTrialSignals(simOut, cfg);
    validateSimulation(simOut, signals, cfg);

    result.metrics = computeTrialMetrics(signals);
    result.metrics.simStopTime = getFinalTime(simOut, signals);
    result.metrics.requestedStopTime = cfg.stopTime;
    result.metrics.torque = computeTorqueMetrics(signals);
    result.simOutputVariables = simOut.who;
    if isfield(cfg, "saveSignals") && cfg.saveSignals
        result.signals = signals;
    end
    result.success = true;
catch err
    result.success = false;
    result.errorMessage = getReport(err, "extended", "hyperlinks", "off");
    result.errorIdentifier = err.identifier;
end

result.elapsedTime = toc(trialTimer);
end

function applyControllerParams(params)
ctrl = evalin("base", "ctrl");
leg = evalin("base", "leg");

ctrl.bandwidthHz = params.bandwidthHz(:);
ctrl.wn = 2 * pi * ctrl.bandwidthHz;
ctrl.zeta = params.zeta(:);
ctrl.Kp = diag(ctrl.wn .^ 2);
ctrl.Kd = diag(2 * ctrl.zeta .* ctrl.wn);

M0 = two_link_leg_dynamics(leg.q0, leg.dq0, leg, "M");
M_eff = diag(M0);
ctrl.pdKp = diag(M_eff .* ctrl.wn .^ 2);
ctrl.pdKd = diag(2 * ctrl.zeta .* M_eff .* ctrl.wn);

assignin("base", "ctrl", ctrl);
end

function configureTrialModel(modelName, cfg)
set_param(modelName, "StopTime", char(string(cfg.stopTime)));
set_param(modelName, "ReturnWorkspaceOutputs", "on");
set_param(modelName, "AlgebraicLoopMsg", "none");

if isfield(cfg, "simulationMode") && strlength(string(cfg.simulationMode)) > 0
    set_param(modelName, "SimulationMode", char(string(cfg.simulationMode)));
end

if isfield(cfg, "fastRestart") && cfg.fastRestart
    set_param(modelName, "FastRestart", "on");
end

if ~isfield(cfg, "disableLogging") || cfg.disableLogging
    set_param(modelName, ...
        "SignalLogging", "off", ...
        "SaveOutput", "off", ...
        "SaveState", "off", ...
        "SaveFinalState", "off", ...
        "SaveTime", "off", ...
        "DSMLogging", "off");
end

if ~isfield(cfg, "disableSDI") || cfg.disableSDI
    disableSdiRecording();
end

for idx = 1:numel(cfg.scopeBlocks)
    set_param(cfg.scopeBlocks(idx), ...
        "SaveToWorkspace", "on", ...
        "SaveName", cfg.scopeSaveNames(idx), ...
        "DataFormat", "Dataset", ...
        "LimitDataPoints", "off");
    setScopeDisplayOff(cfg.scopeBlocks(idx));
end
end

function modelState = captureModelState(modelName, cfg)
modelState.dirty = get_param(modelName, "Dirty");
modelState.stopTime = get_param(modelName, "StopTime");
modelState.returnWorkspaceOutputs = get_param(modelName, "ReturnWorkspaceOutputs");
modelState.algebraicLoopMsg = get_param(modelName, "AlgebraicLoopMsg");
modelState.simulationMode = get_param(modelName, "SimulationMode");
modelState.fastRestart = get_param(modelName, "FastRestart");
modelState.signalLogging = get_param(modelName, "SignalLogging");
modelState.saveOutput = get_param(modelName, "SaveOutput");
modelState.saveState = get_param(modelName, "SaveState");
modelState.saveFinalState = get_param(modelName, "SaveFinalState");
modelState.saveTime = get_param(modelName, "SaveTime");
modelState.dsmLogging = get_param(modelName, "DSMLogging");
modelState.sdi = captureSdiState();
emptyScopeState = struct( ...
    "block", "", ...
    "saveToWorkspace", "", ...
    "saveName", "", ...
    "dataFormat", "", ...
    "limitDataPoints", "", ...
    "openAtSimulationStart", []);
modelState.scopes = repmat(emptyScopeState, numel(cfg.scopeBlocks), 1);

for idx = 1:numel(cfg.scopeBlocks)
    block = cfg.scopeBlocks(idx);
    modelState.scopes(idx).block = block;
    modelState.scopes(idx).saveToWorkspace = get_param(block, "SaveToWorkspace");
    modelState.scopes(idx).saveName = get_param(block, "SaveName");
    modelState.scopes(idx).dataFormat = get_param(block, "DataFormat");
    modelState.scopes(idx).limitDataPoints = get_param(block, "LimitDataPoints");
    modelState.scopes(idx).openAtSimulationStart = getOptionalParam(block, "OpenAtSimulationStart");
end
end

function restoreModelState(modelName, wasLoaded, modelState)
if ~bdIsLoaded(modelName)
    return;
end

try
    if strcmp(get_param(modelName, "FastRestart"), "on")
        set_param(modelName, "FastRestart", "off");
    end

    set_param(modelName, ...
        "StopTime", modelState.stopTime, ...
        "ReturnWorkspaceOutputs", modelState.returnWorkspaceOutputs, ...
        "AlgebraicLoopMsg", modelState.algebraicLoopMsg, ...
        "SimulationMode", modelState.simulationMode, ...
        "SignalLogging", modelState.signalLogging, ...
        "SaveOutput", modelState.saveOutput, ...
        "SaveState", modelState.saveState, ...
        "SaveFinalState", modelState.saveFinalState, ...
        "SaveTime", modelState.saveTime, ...
        "DSMLogging", modelState.dsmLogging);

    for idx = 1:numel(modelState.scopes)
        scope = modelState.scopes(idx);
        set_param(scope.block, ...
            "SaveToWorkspace", scope.saveToWorkspace, ...
            "SaveName", scope.saveName, ...
            "DataFormat", scope.dataFormat, ...
            "LimitDataPoints", scope.limitDataPoints);
        setOptionalParam(scope.block, "OpenAtSimulationStart", ...
            scope.openAtSimulationStart);
    end

    set_param(modelName, "FastRestart", modelState.fastRestart);
    restoreSdiState(modelState.sdi);

    set_param(modelName, "Dirty", modelState.dirty);
catch restoreErr
    warning("run_trial:RestoreFailed", ...
        "Failed to restore model settings: %s", restoreErr.message);
end

if ~wasLoaded && bdIsLoaded(modelName)
    close_system(modelName, 0);
end
end

function signals = extractTrialSignals(simOut, cfg)
signals = struct();
for idx = 1:numel(cfg.signalNames)
    signalName = char(cfg.signalNames(idx));
    dataset = simOut.get(char(cfg.scopeSaveNames(idx)));

    if dataset.numElements < max(cfg.controllerSignalIndex, cfg.referenceSignalIndex)
        error("run_trial:MissingScopeSignals", ...
            "Scope output %s has %d elements.", signalName, dataset.numElements);
    end

    controller = dataset.get(cfg.controllerSignalIndex).Values;
    reference = dataset.get(cfg.referenceSignalIndex).Values;

    signals.(signalName) = struct();
    signals.(signalName).time = controller.Time(:);
    signals.(signalName).controller = controller.Data(:);
    signals.(signalName).reference = reference.Data(:);
end
end

function validateSimulation(simOut, signals, cfg)
finalTime = getFinalTime(simOut, signals);
if finalTime < 0.98 * cfg.stopTime
    error("run_trial:IncompleteSimulation", ...
        "Simulation ended at %.4g s before requested stop time %.4g s.", ...
        finalTime, cfg.stopTime);
end

names = fieldnames(signals);
for idx = 1:numel(names)
    data = [signals.(names{idx}).controller; signals.(names{idx}).reference];
    if isempty(data) || any(~isfinite(data))
        error("run_trial:InvalidSignalData", ...
            "Signal %s is empty or contains NaN/Inf.", names{idx});
    end
end
end

function finalTime = getFinalTime(simOut, signals)
try
    tout = simOut.get("tout");
catch
    tout = [];
end

if isempty(tout)
    finalTime = NaN;
    if nargin >= 2 && isstruct(signals) && isfield(signals, "qh")
        finalTime = signals.qh.time(end);
    end
else
    finalTime = tout(end);
end
end

function value = getOptionalParam(block, paramName)
try
    value = get_param(block, paramName);
catch
    value = [];
end
end

function setOptionalParam(block, paramName, value)
if isempty(value)
    return;
end

try
    set_param(block, paramName, value);
catch
end
end

function setScopeDisplayOff(block)
setOptionalParam(block, "OpenAtSimulationStart", "off");
end

function state = captureSdiState()
state = struct("available", false, "recordData", []);
try
    state.recordData = Simulink.sdi.getRecordData();
    state.available = true;
catch
end
end

function disableSdiRecording()
try
    Simulink.sdi.setRecordData(false);
catch
end
end

function restoreSdiState(state)
if ~isstruct(state) || ~isfield(state, "available") || ~state.available
    return;
end

try
    Simulink.sdi.setRecordData(state.recordData);
catch
end
end

function metrics = computeTrialMetrics(signals)
qError = [
    signals.qh.controller - signals.qh.reference
    signals.qk.controller - signals.qk.reference
    ];
dqError = [
    signals.dqh.controller - signals.dqh.reference
    signals.dqk.controller - signals.dqk.reference
    ];
finalQError = [
    signals.qh.controller(end) - signals.qh.reference(end)
    signals.qk.controller(end) - signals.qk.reference(end)
    ];

metrics = struct();
metrics.positionRms = rms(qError);
metrics.velocityRms = rms(dqError);
metrics.finalPositionErrorNorm = norm(finalQError);
metrics.maxAbsPositionError = max(abs(qError));
metrics.maxAbsVelocityError = max(abs(dqError));
end

function torqueMetrics = computeTorqueMetrics(signals)
leg = evalin("base", "leg");
ctrl = evalin("base", "ctrl");
traj = evalin("base", "traj");

time = signals.qh.time;
torque = zeros(numel(time), 2);

for idx = 1:numel(time)
    q = [signals.qh.controller(idx); signals.qk.controller(idx)];
    dq = [signals.dqh.controller(idx); signals.dqk.controller(idx)];
    [qd, dqd, ddqd] = two_link_leg_reference(time(idx), traj);
    errorQ = qd - q;
    errorDq = dqd - dq;
    [M, C, G] = two_link_leg_dynamics(q, dq, leg);
    v = ddqd + ctrl.Kd * errorDq + ctrl.Kp * errorQ;
    rawTorque = M * v + C + G;
    torque(idx, :) = rawTorque(:).';
end

torqueRatio = abs(torque) ./ reshape(ctrl.tauMax, 1, []);

torqueMetrics = struct();
torqueMetrics.rmsRatio = rms(torqueRatio, "all");
torqueMetrics.maxRatio = max(torqueRatio, [], "all");
torqueMetrics.saturationExcess = max(0, torqueMetrics.maxRatio - 1);
end
