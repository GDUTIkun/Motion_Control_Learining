function configure_base_nmpc_simulink(doSave)
%CONFIGURE_BASE_NMPC_SIMULINK Insert NMPC ahead of symmetric load sharing.

if nargin < 1 || isempty(doSave)
    doSave = true;
end

simulateDir = fileparts(mfilename("fullpath"));
if evalin("base", "exist('baseNmpc', 'var')") ~= 1
    evalin("base", "run(" + quoted(fullfile(simulateDir, "startup.m")) + ");");
end
configure_symmetric_two_leg_simulink(false);
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
wheelStateBlock = subsystem + "/Common Wheel State";
wheelPlanner = subsystem + "/Common Wheel Position LQR";
perLegWrench = subsystem + "/Per-Leg Wrench";

generatedBlocks = find_system(generatedModel, "SearchDepth", 1, ...
    "BlockType", "S-Function");
if numel(generatedBlocks) ~= 1
    error("configure_base_nmpc_simulink:GeneratedBlock", ...
        "Expected exactly one generated S-Function block.");
end

names = [
    "NMPC State Split"
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
disconnectInput(perLegWrench, 1);

stateSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/NMPC State Split", ...
    "Outputs", "[1 8 1]", "Position", [1235, 535, 1240, 635]);
referenceInput = add_block("simulink/Signal Routing/Mux", ...
    subsystem + "/NMPC Reference Input", ...
    "Inputs", "2", "Position", [1285, 520, 1290, 600]);
referenceBlock = add_block(lqrBlock, subsystem + "/NMPC Reference", ...
    "Position", [1340, 535, 1495, 575]);
set_param(referenceBlock, "MATLABFcn", "base_nmpc_reference", ...
    "OutputDimensions", string(baseNmpc.referenceSize), ...
    "SampleTime", "baseNmpc.Ts");
referenceSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/NMPC Reference Split", ...
    "Outputs", sprintf("[11 %d 8]", 11*(baseNmpc.N - 1)), ...
    "Position", [1545, 505, 1550, 595]);
solverBlock = add_block(generatedBlocks{1}, subsystem + "/NMPC Solver", ...
    "Position", [1605, 480, 1825, 670]);
commandMux = add_block("simulink/Signal Routing/Mux", ...
    subsystem + "/NMPC Command Mux", ...
    "Inputs", "4", "Position", [1875, 495, 1880, 625]);
guardBlock = add_block(lqrBlock, subsystem + "/NMPC Command Guard", ...
    "Position", [1930, 535, 2090, 575]);
set_param(guardBlock, "MATLABFcn", "base_nmpc_command", ...
    "OutputDimensions", "4", "SampleTime", "base.Ts");
guardSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/NMPC Guard Split", ...
    "Outputs", "[3 1]", "Position", [2140, 520, 2145, 595]);
fallbackTerminator = add_block("simulink/Sinks/Terminator", ...
    subsystem + "/NMPC Fallback Terminator", ...
    "Position", [2210, 585, 2230, 605]);

connect(subsystem, wheelStateBlock, 1, stateSplit, 1);
connect(subsystem, stateSplit, 1, referenceInput, 1);
connect(subsystem, wheelPlanner, 1, referenceInput, 2);
connect(subsystem, referenceInput, 1, referenceBlock, 1);
connect(subsystem, stateSplit, 2, solverBlock, 1);
connect(subsystem, stateSplit, 2, solverBlock, 2);
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
connect(subsystem, guardSplit, 1, perLegWrench, 1);
connect(subsystem, guardSplit, 2, fallbackTerminator, 1);

logSignal(solverBlock, 1, "nmpcBodyWrench");
logSignal(solverBlock, 2, "nmpcStatus");
logSignal(solverBlock, 3, "nmpcCpuTime");
logSignal(guardSplit, 2, "nmpcFallback");
logSignal(stateSplit, 2, "baseWheelState");
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
