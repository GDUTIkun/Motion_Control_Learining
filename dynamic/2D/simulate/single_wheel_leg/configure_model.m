function configure_model(doSave)
%CONFIGURE_MODEL Set source.slx MATLAB Function names for the new interface.
%
% This helper rewires the controller-facing mux idempotently as:
%
%   base state mux:
%     [t; xB; zB; thetaB; dxB; dzB; dthetaB]
%       -> floating_base_lqr_command
%       -> [FHx_ext; FHz_ext; MBy_des]
%
%   leg controller mux:
%     [t; xB; zB; thetaB; dxB; dzB; dthetaB;
%      qh; qk; qw; dqh; dqk; dqw; FHx_ext; FHz_ext; MBy_des]
%       -> controller_qp
%       -> [tau_h; tau_k; tau_w]

if nargin < 1 || isempty(doSave)
    doSave = true;
end

model = "source";
load_system(model);

setMatlabFcnIfPresent(model, ["floating_base_lqr_command", ...
    "floating_base_lqr_force"], "floating_base_lqr_command", 3);
setMatlabFcnIfPresent(model, "controller_qp", "controller_qp", 3);
wireFloatingBaseControllerInput(model);
setInitialTargets(model);
suppress_scope_windows(model);

sampleTime = getControllerSampleTime(0.005);
set_param(model, "Solver", "ode45", ...
    "MaxStep", sprintf("%.15g", sampleTime), ...
    "RelTol", "1e-3", ...
    "AbsTol", "1e-4");

if doSave
    save_system(model, [], "OverwriteIfChangedOnDisk", true);
end
fprintf("Configured %s for discrete LQR + sampled QP leg control, Ts = %.4f s.\n", ...
    model, sampleTime);
end

function sampleTime = getControllerSampleTime(defaultValue)
sampleTime = defaultValue;
try
    if evalin("base", "exist('base', 'var')") && ...
            evalin("base", "isfield(base, 'Ts')")
        sampleTime = evalin("base", "base.Ts");
    elseif evalin("base", "exist('ctrl', 'var')") && ...
            evalin("base", "isfield(ctrl, 'Ts')")
        sampleTime = evalin("base", "ctrl.Ts");
    end
catch
    sampleTime = defaultValue;
end
sampleTime = double(sampleTime);
if ~isscalar(sampleTime) || ~isfinite(sampleTime) || sampleTime <= 0
    sampleTime = defaultValue;
end
end

function setInitialTargets(model)
subsys = model + "/PD_only";

set_param(subsys + "/Revolute Joint", ...
    "PositionTargetValue", "leg.q0(1)-pi/2");
set_param(subsys + "/Revolute Joint1", ...
    "PositionTargetValue", "leg.q0(2)");
set_param(subsys + "/Revolute Joint2", ...
    "PositionTargetValue", "leg.q0(3)");
set_param(subsys + "/Planar Joint", ...
    "PxPositionTargetSpecify", "on", ...
    "PxPositionTargetPriority", "High", ...
    "PxPositionTargetValue", "base.x0(1)", ...
    "PyPositionTargetSpecify", "on", ...
    "PyPositionTargetPriority", "High", ...
    "PyPositionTargetValue", "base.x0(2)", ...
    "RzPositionTargetSpecify", "on", ...
    "RzPositionTargetPriority", "High", ...
    "RzPositionTargetValue", "base.x0(3)", ...
    "RzPositionTargetValueUnits", "rad", ...
    "PxVelocityTargetSpecify", "on", ...
    "PxVelocityTargetPriority", "High", ...
    "PxVelocityTargetValue", "base.x0(4)", ...
    "PyVelocityTargetSpecify", "on", ...
    "PyVelocityTargetPriority", "High", ...
    "PyVelocityTargetValue", "base.x0(5)", ...
    "RzVelocityTargetSpecify", "on", ...
    "RzVelocityTargetPriority", "High", ...
    "RzVelocityTargetValue", "base.x0(6)", ...
    "RzVelocityTargetValueUnits", "rad/s");
set_param(subsys + "/Rigid Transform", ...
    "TranslationStandardOffset", "base.simscapeWorldYOffset");
end

function wireFloatingBaseControllerInput(model)
subsys = model + "/PD_only";
baseMux = oneBlock(subsys, "Mux");
ctrlMux = oneBlock(subsys, "Mux1");
memory = oneBlock(model, "Controller Input Memory");

baseSrc = cell(7, 1);
for k = 1:7
    baseSrc{k} = sourcePortFromInput(baseMux, k);
end

nCtrlInputs = str2double(get_param(ctrlMux, "Inputs"));
if nCtrlInputs >= 16
    legInputOffset = 7;
    cmdInputOffset = 13;
else
    legInputOffset = 1;
    cmdInputOffset = 7;
end

legSrc = cell(6, 1);
for k = 1:6
    legSrc{k} = sourcePortFromInput(ctrlMux, k + legInputOffset);
end
cmdSrc = cell(3, 1);
for k = 1:3
    cmdSrc{k} = sourcePortFromInput(ctrlMux, k + cmdInputOffset);
end

set_param(ctrlMux, "Inputs", "16");
set_param(memory, "InitialCondition", ...
    "[0; base.x0; leg.q0; leg.dq0; floating_base_lqr_command([0; base.x0])]");

% Controller input:
% [t; xB; zB; thetaB; dxB; dzB; dthetaB;
%  qh; qk; qw; dqh; dqk; dqw; FHx_ext; FHz_ext; MBy_des]
connectPortToInput(baseSrc{1}, ctrlMux, 1);   % clock
for k = 2:7
    connectPortToInput(baseSrc{k}, ctrlMux, k); % base state
end
for k = 1:6
    connectPortToInput(legSrc{k}, ctrlMux, k + 7); % q/dq
end
for k = 1:3
    connectPortToInput(cmdSrc{k}, ctrlMux, k + 13); % LQR command
end
end

function setMatlabFcnIfPresent(model, existingNames, newName, outputDimensions)
blocks = {};
for name = string(existingNames)
    matches = find_system(model, "LookUnderMasks", "all", ...
        "FollowLinks", "on", "BlockType", "MATLABFcn", ...
        "MATLABFcn", name);
    blocks = [blocks; matches(:)]; %#ok<AGROW>
end
for i = 1:numel(blocks)
    set_param(blocks{i}, "MATLABFcn", newName, ...
        "OutputDimensions", string(outputDimensions), "Output1D", "on");
end
end

function block = oneBlock(parent, name)
matches = find_system(parent, "SearchDepth", 1, "Name", name);
if isempty(matches)
    error("configure_model:MissingBlock", "Missing block: %s/%s", parent, name);
end
block = matches{1};
end

function srcPort = sourcePortFromInput(block, inputPort)
handles = get_param(block, "PortHandles");
srcLine = get_param(handles.Inport(inputPort), "Line");
if srcLine == -1
    error("configure_model:UnconnectedSource", ...
        "%s input %d is not connected.", block, inputPort);
end
srcPort = get_param(srcLine, "SrcPortHandle");
if srcPort == -1
    error("configure_model:InvalidSource", ...
        "%s input %d has no source port.", block, inputPort);
end
end

function connectPortToInput(srcPort, dstMux, dstInputPort)
if srcPort == -1
    error("configure_model:InvalidSource", ...
        "Destination %s input %d has an invalid source.", ...
        dstMux, dstInputPort);
end

dstHandles = get_param(dstMux, "PortHandles");
dstLine = get_param(dstHandles.Inport(dstInputPort), "Line");
deleteLineIfValid(dstLine);

parent = get_param(dstMux, "Parent");
try
    add_line(parent, srcPort, dstHandles.Inport(dstInputPort), ...
        "autorouting", "on");
catch err
    if ~contains(err.message, "already")
        rethrow(err);
    end
end
end

function deleteLineIfValid(line)
if isnumeric(line) && isscalar(line) && line ~= -1 && line ~= 0 ...
        && ishandle(line)
    delete_line(line);
end
end
