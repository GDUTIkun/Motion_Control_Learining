function configure_base_nmpc_simulink(doSave)
%CONFIGURE_BASE_NMPC_SIMULINK Insert the generated upper-layer NMPC block.

if nargin < 1 || isempty(doSave)
    doSave = true;
end

simulateDir = fileparts(mfilename("fullpath"));
if evalin("base", "exist('baseNmpc', 'var')") ~= 1
    evalin("base", "run(" + quoted(fullfile(simulateDir, "startup.m")) + ");");
end
baseNmpc = evalin("base", "baseNmpc");
blockFile = fullfile(baseNmpc.generatedDir, ...
    baseNmpc.solverName + "_ocp_solver_simulink_block.slx");
if ~isfile(blockFile)
    error("configure_base_nmpc_simulink:MissingGeneratedBlock", ...
        "Build the solver first: build_base_nmpc_solver(true).");
end

model = "source";
subsystem = model + "/PD_only";
load_system(model);
load_system(blockFile);
[~, generatedModel] = fileparts(blockFile);
cleanupGeneratedModel = onCleanup(@() close_system(generatedModel, 0));

lqrBlock = findOne(subsystem, "BlockType", "MATLABFcn", ...
    "MATLABFcn", "floating_base_lqr_command");
baseStateZoh = findOne(subsystem, "BlockType", "ZeroOrderHold");
lowerCommandDemux = subsystem + "/Demux1";
controllerInputMux = subsystem + "/Mux1";

generatedBlocks = find_system(generatedModel, "SearchDepth", 1, ...
    "BlockType", "S-Function");
if numel(generatedBlocks) ~= 1
    error("configure_base_nmpc_simulink:GeneratedBlock", ...
        "Expected exactly one generated S-Function block.");
end

names = [
    "NMPC State Split"
    "Wheel State Mux"
    "Wheel State"
    "Wheel State Split"
    "Wheel Position LQR"
    "NMPC Reference Input"
    "NMPC Reference"
    "NMPC Reference Split"
    "NMPC Solver"
    "NMPC Command Mux"
    "NMPC Command Guard"
    "NMPC Guard Split"
    "NMPC Fallback Terminator"
];
for i = 1:numel(names)
    deleteBlockIfPresent(subsystem + "/" + names(i));
end
disconnectInput(lowerCommandDemux, 1);
set_param(controllerInputMux, "Inputs", "11");

wheelStateMux = add_block("simulink/Signal Routing/Mux", ...
    subsystem + "/Wheel State Mux", ...
    "Inputs", "7", "Position", [905, 245, 910, 385]);
wheelStateBlock = add_block(lqrBlock, subsystem + "/Wheel State", ...
    "Position", [960, 280, 1115, 320]);
set_param(wheelStateBlock, "MATLABFcn", "wheel_position_state_signal", ...
    "OutputDimensions", "10", "SampleTime", "base.Ts");
wheelStateSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/Wheel State Split", ...
    "Outputs", "[1 8 1]", "Position", [1165, 255, 1170, 345]);
wheelPlanner = add_block(lqrBlock, subsystem + "/Wheel Position LQR", ...
    "Position", [1225, 280, 1380, 320]);
set_param(wheelPlanner, "MATLABFcn", "wheel_position_lqr_reference", ...
    "OutputDimensions", "4", "SampleTime", "base.Ts");
referenceInput = add_block("simulink/Signal Routing/Mux", ...
    subsystem + "/NMPC Reference Input", ...
    "Inputs", "2", "Position", [1425, 255, 1430, 325]);
referenceBlock = add_block(lqrBlock, subsystem + "/NMPC Reference", ...
    "Position", [1480, 270, 1635, 310]);
set_param(referenceBlock, "MATLABFcn", "base_nmpc_reference", ...
    "OutputDimensions", string(baseNmpc.referenceSize), ...
    "SampleTime", "baseNmpc.Ts");
referenceSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/NMPC Reference Split", ...
    "Outputs", sprintf("[11 %d 8]", 11*(baseNmpc.N - 1)), ...
    "Position", [1685, 245, 1690, 335]);
solverBlock = add_block(generatedBlocks{1}, subsystem + "/NMPC Solver", ...
    "Position", [1745, 235, 1965, 425]);
commandMux = add_block("simulink/Signal Routing/Mux", ...
    subsystem + "/NMPC Command Mux", ...
    "Inputs", "4", "Position", [2015, 250, 2020, 380]);
guardBlock = add_block(lqrBlock, subsystem + "/NMPC Command Guard", ...
    "Position", [2070, 290, 2230, 330]);
set_param(guardBlock, "MATLABFcn", "base_nmpc_command", ...
    "OutputDimensions", "4", "SampleTime", "base.Ts");
guardSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/NMPC Guard Split", ...
    "Outputs", "[3 1]", "Position", [2280, 280, 2285, 345]);
fallbackTerminator = add_block("simulink/Sinks/Terminator", ...
    subsystem + "/NMPC Fallback Terminator", ...
    "Position", [2350, 345, 2370, 365]);

connect(subsystem, baseStateZoh, 1, wheelStateMux, 1);
for idx = 2:7
    branchInputSource(subsystem, controllerInputMux, idx, wheelStateMux, idx);
end
connect(subsystem, wheelStateMux, 1, wheelStateBlock, 1);
connect(subsystem, wheelStateBlock, 1, wheelStateSplit, 1);
connect(subsystem, wheelStateBlock, 1, wheelPlanner, 1);
connect(subsystem, wheelStateSplit, 1, referenceInput, 1);
connect(subsystem, wheelPlanner, 1, referenceInput, 2);
connect(subsystem, referenceInput, 1, referenceBlock, 1);
connect(subsystem, wheelStateSplit, 2, solverBlock, 1);
connect(subsystem, wheelStateSplit, 2, solverBlock, 2);
connect(subsystem, referenceBlock, 1, referenceSplit, 1);
connect(subsystem, referenceSplit, 1, solverBlock, 3);
connect(subsystem, referenceSplit, 2, solverBlock, 4);
connect(subsystem, referenceSplit, 3, solverBlock, 5);
connect(subsystem, solverBlock, 1, commandMux, 1);
connect(subsystem, solverBlock, 2, commandMux, 2);
connect(subsystem, solverBlock, 3, commandMux, 3);
connect(subsystem, lqrBlock, 1, commandMux, 4);
connect(subsystem, commandMux, 1, guardBlock, 1);
connect(subsystem, guardBlock, 1, guardSplit, 1);
connect(subsystem, guardSplit, 1, lowerCommandDemux, 1);
connect(subsystem, guardSplit, 2, fallbackTerminator, 1);
connect(subsystem, wheelPlanner, 1, controllerInputMux, 11);

logSignal(solverBlock, 1, "nmpcBodyWrench");
logSignal(solverBlock, 2, "nmpcStatus");
logSignal(solverBlock, 3, "nmpcCpuTime");
logSignal(guardSplit, 2, "nmpcFallback");
logSignal(wheelStateSplit, 2, "baseWheelState");
logSignal(wheelPlanner, 1, "wheelPositionLqrReference");

set_param(model, "SimulationCommand", "update");
if doSave
    save_system(model, [], "OverwriteIfChangedOnDisk", true);
end
fprintf("Configured upper-layer NMPC S-Function in %s.\n", model);
end

function value = quoted(pathValue)
value = "'" + replace(string(pathValue), "'", "''") + "'";
end

function block = findOne(parent, varargin)
blocks = find_system(parent, "SearchDepth", 1, varargin{:});
blocks = setdiff(string(blocks), string(parent), "stable");
if numel(blocks) ~= 1
    error("configure_base_nmpc_simulink:BlockLookup", ...
        "Expected one matching block under %s, found %d.", parent, numel(blocks));
end
block = blocks(1);
end

function deleteBlockIfPresent(block)
if getSimulinkBlockHandle(block) ~= -1
    delete_block(block);
end
end

function disconnectInput(block, port)
handles = get_param(block, "PortHandles");
line = get_param(handles.Inport(port), "Line");
if line ~= -1
    delete_line(line);
end
end

function connect(parent, srcBlock, srcPort, dstBlock, dstPort)
srcHandles = get_param(srcBlock, "PortHandles");
dstHandles = get_param(dstBlock, "PortHandles");
disconnectPort(dstHandles.Inport(dstPort));
add_line(parent, srcHandles.Outport(srcPort), dstHandles.Inport(dstPort), ...
    "autorouting", "on");
end

function branchInputSource(parent, sourceBlock, sourceInput, dstBlock, dstInput)
sourceHandles = get_param(sourceBlock, "PortHandles");
sourceLine = get_param(sourceHandles.Inport(sourceInput), "Line");
if sourceLine == -1
    error("configure_base_nmpc_simulink:MissingSource", ...
        "Input %d of %s is not connected.", sourceInput, sourceBlock);
end
sourcePort = get_param(sourceLine, "SrcPortHandle");
destinationHandles = get_param(dstBlock, "PortHandles");
disconnectPort(destinationHandles.Inport(dstInput));
add_line(parent, sourcePort, destinationHandles.Inport(dstInput), ...
    "autorouting", "on");
end

function disconnectPort(portHandle)
line = get_param(portHandle, "Line");
if line ~= -1
    delete_line(line);
end
end

function logSignal(block, port, name)
handles = get_param(block, "PortHandles");
outport = handles.Outport(port);
line = get_param(outport, "Line");
if line == -1
    error("configure_base_nmpc_simulink:MissingLine", ...
        "Cannot log unconnected signal %s.", name);
end
set_param(line, "Name", name);
set_param(outport, "DataLogging", "on", ...
    "DataLoggingNameMode", "Custom", "DataLoggingName", name);
end
