function result = run_trial(params, cfg)
%RUN_TRIAL Run one source.slx QP wheel-leg trial for a candidate set.

trialTimer = tic;
result = struct();
result.success = false;
result.errorMessage = "";
result.params = params;

try
    addpath(cfg.simDir);
    matlabState = configureHeadlessMatlab(cfg);
    runStartupInBase(cfg.startupScript);

    modelName = char(cfg.modelName);
    wasLoaded = bdIsLoaded(modelName);
    load_system(cfg.modelPath);

    modelState = captureModelState(modelName, cfg);
    cleanup = onCleanup(@() restoreTrialState(modelName, wasLoaded, ...
        modelState, matlabState));

    applyControllerParams(params);
    configureTrialModel(modelName, cfg);

    simOut = runSimulationWithFallback(modelName, cfg);
    signals = extractTrialSignals(simOut, cfg);
    validateSimulation(simOut, signals, cfg);

    result.metrics = computeTrialMetrics(signals, params);
    result.metrics.simStopTime = getFinalTime(simOut, signals);
    result.metrics.requestedStopTime = cfg.stopTime;
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

function matlabState = configureHeadlessMatlab(cfg)
matlabState = struct("defaultFigureVisible", []);

if isfield(cfg, "disableGraphics") && cfg.disableGraphics
    try
        matlabState.defaultFigureVisible = get(0, "DefaultFigureVisible");
        set(0, "DefaultFigureVisible", "off");
    catch
    end
end

if ~isfield(cfg, "disableSDI") || cfg.disableSDI
    try
        Simulink.sdi.clear;
    catch
    end
end
end

function runStartupInBase(startupScript)
assignin("base", "calibrationStartupScript", startupScript);
evalin("base", "run(calibrationStartupScript); clear calibrationStartupScript");
end

function restoreTrialState(modelName, wasLoaded, modelState, matlabState)
restoreModelState(modelName, wasLoaded, modelState);

if isstruct(matlabState) && isfield(matlabState, "defaultFigureVisible") ...
        && ~isempty(matlabState.defaultFigureVisible)
    try
        set(0, "DefaultFigureVisible", matlabState.defaultFigureVisible);
    catch
    end
end
end

function applyControllerParams(params)
ctrl = evalin("base", "ctrl");

ctrl.bandwidthHz = params.bandwidthHz(:);
ctrl.wn = 2 * pi * ctrl.bandwidthHz;
ctrl.zeta = params.zeta(:);
ctrl.Kp = diag(ctrl.wn .^ 2);
ctrl.Kd = diag(2 * ctrl.zeta .* ctrl.wn);
ctrl.constraintVelocityGain = params.constraintVelocityGain;
ctrl.qpWtau = params.qpWtau(:);
ctrl.qpWFc = params.qpWFc(:);

assignin("base", "ctrl", ctrl);
end

function configureTrialModel(modelName, cfg)
set_param(modelName, "StopTime", char(string(cfg.stopTime)));
set_param(modelName, ...
    "ReturnWorkspaceOutputs", "on", ...
    "AlgebraicLoopMsg", "none");

if isfield(cfg, "simulationMode") && strlength(string(cfg.simulationMode)) > 0
    set_param(modelName, "SimulationMode", char(string(cfg.simulationMode)));
end

setTopController(modelName, "controller_qp");

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
        "DSMLogging", "off", ...
        "InspectSignalLogs", "off", ...
        "LimitDataPoints", "on", ...
        "MaxDataPoints", char(string(getCfgValue(cfg, "maxLoggedPoints", 15000))), ...
        "Decimation", char(string(getCfgValue(cfg, "scopeDecimation", 2))));
    setOptionalParam(modelName, "SimscapeLogType", "none");
end

if ~isfield(cfg, "disableSDI") || cfg.disableSDI
    disableSdiRecording();
end

if isfield(cfg, "disableGraphics") && cfg.disableGraphics
    setOptionalParam(modelName, "SimMechanicsOpenEditorOnUpdate", "off");
end

for idx = 1:numel(cfg.scopeBlocks)
    set_param(cfg.scopeBlocks(idx), ...
        "SaveToWorkspace", "on", ...
        "SaveName", cfg.scopeSaveNames(idx), ...
        "DataFormat", "Dataset", ...
        "LimitDataPoints", "on", ...
        "MaxDataPoints", char(string(getCfgValue(cfg, "maxLoggedPoints", 15000))), ...
        "Decimation", char(string(getCfgValue(cfg, "scopeDecimation", 2))));
    setScopeDisplayOff(cfg.scopeBlocks(idx));
end
end

function setTopController(modelName, controllerFcn)
blocks = find_system(modelName, "LookUnderMasks", "all", ...
    "FollowLinks", "on", "BlockType", "MATLABFcn", ...
    "OutputDimensions", "3");
if isempty(blocks)
    error("run_trial:MissingControllerBlock", ...
        "Could not find a 3-output Interpreted MATLAB Function controller.");
end

set_param(blocks{1}, "MATLABFcn", controllerFcn, ...
    "OutputDimensions", "3", "Output1D", "on");
end

function simOut = runSimulationWithFallback(modelName, cfg)
try
    simOut = sim(modelName, "ReturnWorkspaceOutputs", "on");
catch err
    if isfield(cfg, "fastRestart") && cfg.fastRestart
        warning("run_trial:FastRestartFallback", ...
            "Simulation failed with Fast Restart enabled. Retrying with Fast Restart off. Original error: %s", ...
            err.message);
        try
            set_param(modelName, "FastRestart", "off");
        catch
        end
        simOut = sim(modelName, "ReturnWorkspaceOutputs", "on");
    else
        rethrow(err);
    end
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
modelState.inspectSignalLogs = getOptionalParam(modelName, "InspectSignalLogs");
modelState.limitDataPoints = getOptionalParam(modelName, "LimitDataPoints");
modelState.maxDataPoints = getOptionalParam(modelName, "MaxDataPoints");
modelState.decimation = getOptionalParam(modelName, "Decimation");
modelState.simscapeLogType = getOptionalParam(modelName, "SimscapeLogType");
modelState.simMechanicsOpenEditorOnUpdate = ...
    getOptionalParam(modelName, "SimMechanicsOpenEditorOnUpdate");
modelState.sdi = captureSdiState();
modelState.controller = captureTopController(modelName);

emptyScopeState = struct( ...
    "block", "", ...
    "saveToWorkspace", "", ...
    "saveName", "", ...
    "dataFormat", "", ...
    "limitDataPoints", "", ...
    "maxDataPoints", "", ...
    "decimation", "", ...
    "openAtSimulationStart", []);
modelState.scopes = repmat(emptyScopeState, numel(cfg.scopeBlocks), 1);

for idx = 1:numel(cfg.scopeBlocks)
    block = cfg.scopeBlocks(idx);
    modelState.scopes(idx).block = block;
    modelState.scopes(idx).saveToWorkspace = get_param(block, "SaveToWorkspace");
    modelState.scopes(idx).saveName = get_param(block, "SaveName");
    modelState.scopes(idx).dataFormat = get_param(block, "DataFormat");
    modelState.scopes(idx).limitDataPoints = get_param(block, "LimitDataPoints");
    modelState.scopes(idx).maxDataPoints = get_param(block, "MaxDataPoints");
    modelState.scopes(idx).decimation = get_param(block, "Decimation");
    modelState.scopes(idx).openAtSimulationStart = getOptionalParam(block, "OpenAtSimulationStart");
end
end

function controllerState = captureTopController(modelName)
controllerState = struct("block", "", "matlabFcn", "", ...
    "outputDimensions", "", "output1D", "");
blocks = find_system(modelName, "LookUnderMasks", "all", ...
    "FollowLinks", "on", "BlockType", "MATLABFcn", ...
    "OutputDimensions", "3");
if isempty(blocks)
    return;
end
controllerState.block = blocks{1};
controllerState.matlabFcn = get_param(blocks{1}, "MATLABFcn");
controllerState.outputDimensions = get_param(blocks{1}, "OutputDimensions");
controllerState.output1D = get_param(blocks{1}, "Output1D");
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
    setOptionalParam(modelName, "InspectSignalLogs", modelState.inspectSignalLogs);
    setOptionalParam(modelName, "LimitDataPoints", modelState.limitDataPoints);
    setOptionalParam(modelName, "MaxDataPoints", modelState.maxDataPoints);
    setOptionalParam(modelName, "Decimation", modelState.decimation);
    setOptionalParam(modelName, "SimscapeLogType", modelState.simscapeLogType);
    setOptionalParam(modelName, "SimMechanicsOpenEditorOnUpdate", ...
        modelState.simMechanicsOpenEditorOnUpdate);

    if isfield(modelState, "controller") && strlength(string(modelState.controller.block)) > 0
        set_param(modelState.controller.block, ...
            "MATLABFcn", modelState.controller.matlabFcn, ...
            "OutputDimensions", modelState.controller.outputDimensions, ...
            "Output1D", modelState.controller.output1D);
    end

    for idx = 1:numel(modelState.scopes)
        scope = modelState.scopes(idx);
        set_param(scope.block, ...
            "SaveToWorkspace", scope.saveToWorkspace, ...
            "SaveName", scope.saveName, ...
            "DataFormat", scope.dataFormat, ...
            "LimitDataPoints", scope.limitDataPoints, ...
            "MaxDataPoints", scope.maxDataPoints, ...
            "Decimation", scope.decimation);
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

    requiredElements = max(cfg.actualSignalIndex, cfg.referenceSignalIndex);
    if dataset.numElements < requiredElements
        error("run_trial:MissingScopeSignals", ...
            "Scope output %s has %d elements.", signalName, dataset.numElements);
    end

    actual = dataset.get(cfg.actualSignalIndex).Values;
    reference = dataset.get(cfg.referenceSignalIndex).Values;

    signals.(signalName) = struct();
    signals.(signalName).time = actual.Time(:);
    signals.(signalName).actual = actual.Data(:);
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
    data = [signals.(names{idx}).actual; signals.(names{idx}).reference];
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

function metrics = computeTrialMetrics(signals, params)
legQError = [
    signals.qh.actual - signals.qh.reference
    signals.qk.actual - signals.qk.reference
    ];
wheelQError = signals.qw.actual - signals.qw.reference;
legDqError = [
    signals.dqh.actual - signals.dqh.reference
    signals.dqk.actual - signals.dqk.reference
    ];
wheelDqError = signals.dqw.actual - signals.dqw.reference;
finalLegQError = [
    signals.qh.actual(end) - signals.qh.reference(end)
    signals.qk.actual(end) - signals.qk.reference(end)
    ];
finalWheelQError = signals.qw.actual(end) - signals.qw.reference(end);

metrics = struct();
metrics.legPositionRms = rms(legQError);
metrics.wheelPositionRms = rms(wheelQError);
metrics.legVelocityRms = rms(legDqError);
metrics.wheelVelocityRms = rms(wheelDqError);
metrics.finalLegPositionErrorNorm = norm(finalLegQError);
metrics.finalWheelPositionErrorAbs = abs(finalWheelQError);
metrics.maxAbsLegPositionError = max(abs(legQError));
metrics.maxAbsWheelPositionError = max(abs(wheelQError));
metrics.bandwidthNorm = norm(params.bandwidthHz(:));
end

function value = getCfgValue(cfg, fieldName, defaultValue)
if isfield(cfg, fieldName) && ~isempty(cfg.(fieldName))
    value = cfg.(fieldName);
else
    value = defaultValue;
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
