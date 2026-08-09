function configure_discrete_controller_timing(doSave)
%CONFIGURE_DISCRETE_CONTROLLER_TIMING Align sampled LQR/QP execution.
%
% This keeps the current 16D Simulink interface, but fixes the timing:
%   - sample the full 16D controller input with a top-level ZOH;
%   - sample q/dq before they are assembled into the QP input vector;
%   - run the MATLAB Function blocks at the controller sample time.

if nargin < 1 || isempty(doSave)
    doSave = true;
end

model = "source";
subsys = model + "/PD_only";
load_system(model);

TsExpr = "base.Ts";
sampleTime = getControllerSampleTime(0.005);

setTopLevelInputZoh(model, TsExpr);
restoreRawLegStateForQp(subsys);
setControllerSampleTimes(model, TsExpr);
suppress_scope_windows(model);

set_param(model, "Solver", "ode45", ...
    "MaxStep", sprintf("%.15g", sampleTime), ...
    "RelTol", "1e-3", ...
    "AbsTol", "1e-4");

if doSave
    save_system(model, [], "OverwriteIfChangedOnDisk", true);
end

fprintf("Configured discrete controller timing for %s, Ts = %.4f s.\n", ...
    model, sampleTime);
end

function setTopLevelInputZoh(model, TsExpr)
plant = model + "/PD_only";
matches = find_system(model, "SearchDepth", 1, "BlockType", "MATLABFcn", ...
    "MATLABFcn", "controller_qp_signal");
if isempty(matches)
    matches = find_system(model, "SearchDepth", 1, "BlockType", "MATLABFcn", ...
        "MATLABFcn", "controller_qp");
end
if isempty(matches)
    error("configure_discrete_controller_timing:MissingController", ...
        "Could not find the top-level QP MATLAB Function block.");
end
controller = matches{1};
zoh = ensureBlock("simulink/Discrete/Zero-Order Hold", ...
    model + "/Controller Input ZOH", [215 185 255 225]);
set_param(zoh, "SampleTime", TsExpr);

plantOut = get_param(plant, "PortHandles");
zohHandles = get_param(zoh, "PortHandles");
controllerIn = get_param(controller, "PortHandles");
deleteLineIfValid(get_param(zohHandles.Inport(1), "Line"));
deleteLineIfValid(get_param(controllerIn.Inport(1), "Line"));
addLineIfMissing(model, plantOut.Outport(1), zohHandles.Inport(1));
addLineIfMissing(model, zohHandles.Outport(1), controllerIn.Inport(1));

oldBlocks = [
    model + "/Controller Input Unit Delay";
    model + "/Controller Input Memory"
];
for i = 1:numel(oldBlocks)
    if isBlock(oldBlocks(i))
        delete_block(oldBlocks(i));
    end
end
end

function restoreRawLegStateForQp(subsys)
ctrlMux = subsys + "/Mux1";
set_param(ctrlMux, "Inputs", "10");

srcBlocks = [
    subsys + "/Zero-Order" + newline + "Hold";
    subsys + "/Fcn";
    subsys + "/PS-Simulink" + newline + "Converter1";
    subsys + "/PS-Simulink" + newline + "Converter2";
    subsys + "/PS-Simulink" + newline + "Converter9";
    subsys + "/PS-Simulink" + newline + "Converter10";
    subsys + "/PS-Simulink" + newline + "Converter11";
];

for k = 1:numel(srcBlocks)
    connectBlockOutToInput(subsys, srcBlocks(k), 1, ctrlMux, k);
end

demux = subsys + "/Demux1";
for k = 1:3
    connectBlockOutToInput(subsys, demux, k, ctrlMux, k + 7);
end

blocksToDelete = [
    subsys + "/QP qdq Demux";
    subsys + "/QP qdq ZOH";
    subsys + "/QP qdq Mux"
];
for i = 1:numel(blocksToDelete)
    if isBlock(blocksToDelete(i))
        delete_block(blocksToDelete(i));
    end
end
end

function connectBlockOutToInput(parent, srcBlock, srcPort, dstBlock, dstPort)
srcBlock = char(srcBlock);
srcHandles = get_param(srcBlock, "PortHandles");
dstHandles = get_param(dstBlock, "PortHandles");
deleteLineIfValid(get_param(dstHandles.Inport(dstPort), "Line"));
addLineIfMissing(parent, srcHandles.Outport(srcPort), dstHandles.Inport(dstPort));
end

function setControllerSampleTimes(model, TsExpr)
blocks = find_system(model, "LookUnderMasks", "all", ...
    "FollowLinks", "on", "BlockType", "MATLABFcn");
for i = 1:numel(blocks)
    fcn = string(get_param(blocks{i}, "MATLABFcn"));
    if fcn == "floating_base_lqr_command" || fcn == "controller_qp"
        set_param(blocks{i}, "SampleTime", TsExpr);
    end
end
end

function block = ensureBlock(libraryBlock, block, position)
block = char(block);
slashIdx = find(block == '/', 1, 'last');
parent = block(1:slashIdx-1);
name = block(slashIdx+1:end);
if isempty(find_system(parent, "SearchDepth", 1, "Name", name))
    add_block(libraryBlock, block, "Position", position);
else
    set_param(block, "Position", position);
end
end

function tf = isBlock(block)
block = char(block);
slashIdx = find(block == '/', 1, 'last');
parent = block(1:slashIdx-1);
name = block(slashIdx+1:end);
tf = ~isempty(find_system(parent, "SearchDepth", 1, "Name", name));
end

function srcPort = sourcePortFromInput(block, inputPort)
handles = get_param(block, "PortHandles");
srcLine = get_param(handles.Inport(inputPort), "Line");
if srcLine == -1
    error("configure_discrete_controller_timing:UnconnectedInput", ...
        "%s input %d is not connected.", block, inputPort);
end
srcPort = get_param(srcLine, "SrcPortHandle");
if srcPort == -1
    error("configure_discrete_controller_timing:InvalidSource", ...
        "%s input %d has no source port.", block, inputPort);
end
end

function addLineIfMissing(parent, srcPort, dstPort)
try
    add_line(parent, srcPort, dstPort, "autorouting", "on");
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
