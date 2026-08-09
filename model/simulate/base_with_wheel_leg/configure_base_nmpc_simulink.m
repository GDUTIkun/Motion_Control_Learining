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

generatedBlocks = find_system(generatedModel, "SearchDepth", 1, ...
    "BlockType", "S-Function");
if numel(generatedBlocks) ~= 1
    error("configure_base_nmpc_simulink:GeneratedBlock", ...
        "Expected exactly one generated S-Function block.");
end

names = [
    "NMPC State Split"
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

stateSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/NMPC State Split", ...
    "Outputs", "[1 6]", "Position", [905, 70, 910, 150]);
referenceBlock = add_block(lqrBlock, subsystem + "/NMPC Reference", ...
    "Position", [970, 55, 1125, 95]);
set_param(referenceBlock, "MATLABFcn", "base_nmpc_reference", ...
    "OutputDimensions", string(baseNmpc.referenceSize), ...
    "SampleTime", "baseNmpc.Ts");
referenceSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/NMPC Reference Split", ...
    "Outputs", sprintf("[9 %d 6]", 9*(baseNmpc.N - 1)), ...
    "Position", [1170, 40, 1175, 125]);
solverBlock = add_block(generatedBlocks{1}, subsystem + "/NMPC Solver", ...
    "Position", [1240, 60, 1460, 250]);
commandMux = add_block("simulink/Signal Routing/Mux", ...
    subsystem + "/NMPC Command Mux", ...
    "Inputs", "4", "Position", [1515, 75, 1520, 205]);
guardBlock = add_block(lqrBlock, subsystem + "/NMPC Command Guard", ...
    "Position", [1570, 115, 1730, 155]);
set_param(guardBlock, "MATLABFcn", "base_nmpc_command", ...
    "OutputDimensions", "4", "SampleTime", "base.Ts");
guardSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/NMPC Guard Split", ...
    "Outputs", "[3 1]", "Position", [1780, 105, 1785, 170]);
fallbackTerminator = add_block("simulink/Sinks/Terminator", ...
    subsystem + "/NMPC Fallback Terminator", ...
    "Position", [1850, 170, 1870, 190]);

connect(subsystem, baseStateZoh, 1, stateSplit, 1);
connect(subsystem, stateSplit, 1, referenceBlock, 1);
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
connect(subsystem, guardSplit, 1, lowerCommandDemux, 1);
connect(subsystem, guardSplit, 2, fallbackTerminator, 1);

logSignal(solverBlock, 1, "nmpcBodyWrench");
logSignal(solverBlock, 2, "nmpcStatus");
logSignal(solverBlock, 3, "nmpcCpuTime");
logSignal(guardSplit, 2, "nmpcFallback");

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
