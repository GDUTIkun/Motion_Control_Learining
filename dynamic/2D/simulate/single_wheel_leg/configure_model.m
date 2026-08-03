function configure_model()
%CONFIGURE_MODEL Wire source.slx for LQR-to-QP force-command validation.
%
% The Rectangular Joint is left dynamically free. The upper MATLAB Function
% computes an LQR body-force command and passes the body-on-leg reaction
% force into the QP inverse-dynamics controller as FH_ext.

model = "source";
subsys = model + "/PD_only";

load_system(model);

rect = oneBlock(subsys, "Rectangular Joint");
hipFcn = oneBlock(subsys, "Hip Command F_H");
hipDemux = oneBlock(subsys, "Hip Command Demux");
ctrlMux = oneBlock(subsys, "Mux");
clock = oneBlock(subsys, "Clock");
psHipX = oneBlock(subsys, "PS-Simulink Converter4");
psHipY = oneBlock(subsys, "PS-Simulink Converter3");
psWheel = oneBlock(subsys, "PS-Simulink Converter2");
wheelVel = oneBlock(subsys, "Derivative2");
hipYVel = oneBlock(subsys, "Derivative3");
simPsHipX = optionalBlock(subsys, "Simulink-PS Converter2");
simPsHipY = optionalBlock(subsys, "Simulink-PS Converter3");

if strlength(simPsHipX) > 0 && strlength(simPsHipY) > 0
    set_param(rect, ...
        "PxTorqueActuationMode", "InputTorque", ...
        "PyTorqueActuationMode", "InputTorque", ...
        "PxMotionActuationMode", "ComputedMotion", ...
        "PyMotionActuationMode", "ComputedMotion", ...
        "PxSensePosition", "on", ...
        "PySensePosition", "on");
else
    set_param(rect, ...
        "PxTorqueActuationMode", "NoTorque", ...
        "PyTorqueActuationMode", "NoTorque", ...
        "PxMotionActuationMode", "ComputedMotion", ...
        "PyMotionActuationMode", "ComputedMotion", ...
        "PxSensePosition", "on", ...
        "PySensePosition", "on");
end

set_param(hipFcn, "MATLABFcn", "floating_base_lqr_force", ...
    "OutputDimensions", "4", "Output1D", "on");
set_param(hipDemux, "Outputs", "4");
set_param(ctrlMux, "Inputs", "9");

deleteOutportLine(hipDemux, 1);
deleteOutportLine(hipDemux, 2);

if strlength(simPsHipX) > 0 && strlength(simPsHipY) > 0
    deletePhysicalLines(simPsHipX);
    deletePhysicalLines(simPsHipY);
    deleteInportLine(simPsHipX, 1);
    deleteInportLine(simPsHipY, 1);

    hipFxZero = ensureConstant(subsys, "Hip Fx Zero", [0 110 30 140]);
    hipFyZero = ensureConstant(subsys, "Hip Fy Zero", [0 145 30 175]);
    connect(hipFxZero, 1, simPsHipX, 1);
    connect(hipFyZero, 1, simPsHipY, 1);
    connectPhysical(simPsHipX, "RConn", 1, rect, "LConn", 2);
    connectPhysical(simPsHipY, "RConn", 1, rect, "LConn", 3);
end

% q = [qh;qk;qw] and dq = [dqh;dqk;dqw]. The wheel angle must come from the
% wheel revolute joint, not from the hip rectangular joint position.
deleteInportLine(ctrlMux, 4);
connect(psWheel, 1, ctrlMux, 4);
deleteInportLine(wheelVel, 1);
connect(psWheel, 1, wheelVel, 1);
deleteInportLine(ctrlMux, 7);
connect(wheelVel, 1, ctrlMux, 7);

hipMuxPath = subsys + "/Hip PD Mux";
if ~bdIsLoaded(model) || isempty(find_system(subsys, "SearchDepth", 1, "Name", "Hip PD Mux"))
    add_block("simulink/Signal Routing/Mux", hipMuxPath, ...
        "Inputs", "5", "Position", [165 235 170 345]);
else
    set_param(hipMuxPath, "Inputs", "5");
end

hipXVelPath = subsys + "/Derivative4";
if isempty(find_system(subsys, "SearchDepth", 1, "Name", "Derivative4"))
    add_block("simulink/Continuous/Derivative", hipXVelPath, ...
        "Position", [110 175 140 205]);
end

deleteInportLine(hipFcn, 1);
connect(clock, 1, hipMuxPath, 1);
connect(psHipX, 1, hipMuxPath, 2);
connect(psHipY, 1, hipMuxPath, 3);

deleteInportLine(hipXVelPath, 1);
connect(psHipX, 1, hipXVelPath, 1);
connect(hipXVelPath, 1, hipMuxPath, 4);

deleteInportLine(hipYVel, 1);
connect(psHipY, 1, hipYVel, 1);
connect(hipYVel, 1, hipMuxPath, 5);

connect(hipMuxPath, 1, hipFcn, 1);

% Controller force-command ports: [FHx_ext; FHz_ext].
deleteInportLine(ctrlMux, 8);
deleteInportLine(ctrlMux, 9);
connect(hipDemux, 3, ctrlMux, 8);
connect(hipDemux, 4, ctrlMux, 9);

setTopControllerToQp(model);

set_param(model, "Solver", "ode23t", ...
    "MaxStep", "0.005", ...
    "RelTol", "1e-3", ...
    "AbsTol", "1e-4");

save_system(model);
fprintf("Configured %s for floating-base LQR force command + QP leg control.\n", model);
end

function setTopControllerToQp(model)
blocks = find_system(model, "LookUnderMasks", "all", ...
    "FollowLinks", "on", "BlockType", "MATLABFcn", ...
    "MATLABFcn", "controller");
blocks = [blocks; find_system(model, "LookUnderMasks", "all", ...
    "FollowLinks", "on", "BlockType", "MATLABFcn", ...
    "MATLABFcn", "controller_constraint")];
blocks = [blocks; find_system(model, "LookUnderMasks", "all", ...
    "FollowLinks", "on", "BlockType", "MATLABFcn", ...
    "MATLABFcn", "controller_qp")];
if ~isempty(blocks)
    set_param(blocks{1}, "MATLABFcn", "controller_qp", ...
        "OutputDimensions", "3", "Output1D", "on");
end
end

function block = ensureConstant(parent, name, position)
matches = find_system(parent, "SearchDepth", 1, "Name", name);
if isempty(matches)
    block = parent + "/" + name;
    add_block("simulink/Sources/Constant", block, ...
        "Value", "0", "Position", position);
else
    block = matches{1};
    set_param(block, "Value", "0");
end
end

function block = oneBlock(parent, name)
matches = find_system(parent, "SearchDepth", 1, "Name", name);
if isempty(matches)
    error("configure_model:MissingBlock", "Missing block: %s/%s", parent, name);
end
block = matches{1};
end

function block = optionalBlock(parent, name)
matches = find_system(parent, "SearchDepth", 1, "Name", name);
if isempty(matches)
    block = "";
else
    block = string(matches{1});
end
end

function connect(srcBlock, srcPort, dstBlock, dstPort)
srcHandles = get_param(srcBlock, "PortHandles");
dstHandles = get_param(dstBlock, "PortHandles");
parent = get_param(srcBlock, "Parent");
line = get_param(dstHandles.Inport(dstPort), "Line");
deleteLineIfValid(line);
try
    add_line(parent, srcHandles.Outport(srcPort), ...
        dstHandles.Inport(dstPort), "autorouting", "on");
catch err
    if ~contains(err.message, "already")
        rethrow(err);
    end
end
end

function connectPhysical(srcBlock, srcKind, srcPort, dstBlock, dstKind, dstPort)
srcHandles = get_param(srcBlock, "PortHandles");
dstHandles = get_param(dstBlock, "PortHandles");
parent = get_param(srcBlock, "Parent");
line = get_param(dstHandles.(dstKind)(dstPort), "Line");
deleteLineIfValid(line);
try
    add_line(parent, srcHandles.(srcKind)(srcPort), ...
        dstHandles.(dstKind)(dstPort), "autorouting", "on");
catch err
    if ~contains(err.message, "already")
        rethrow(err);
    end
end
end

function deleteInportLine(block, portIndex)
handles = get_param(block, "PortHandles");
if numel(handles.Inport) < portIndex
    return;
end
line = get_param(handles.Inport(portIndex), "Line");
deleteLineIfValid(line);
end

function deleteOutportLine(block, portIndex)
handles = get_param(block, "PortHandles");
if numel(handles.Outport) < portIndex
    return;
end
line = get_param(handles.Outport(portIndex), "Line");
deleteLineIfValid(line);
end

function deletePhysicalLines(block)
handles = get_param(block, "PortHandles");
ports = [handles.LConn(:); handles.RConn(:)];
for idx = 1:numel(ports)
    line = get_param(ports(idx), "Line");
    deleteLineIfValid(line);
end
end

function deleteLineIfValid(line)
if isnumeric(line) && isscalar(line) && line ~= -1 && line ~= 0 ...
        && ishandle(line)
    delete_line(line);
end
end
