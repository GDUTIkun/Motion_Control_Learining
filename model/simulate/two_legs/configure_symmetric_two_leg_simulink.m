function configure_symmetric_two_leg_simulink(doSave)
%CONFIGURE_SYMMETRIC_TWO_LEG_SIMULINK Wire one floating-base QP to both legs.
%
% The upper controller produces one total body wrench. Both leg states,
% both rolling contacts, and both actuator vectors enter one coupled QP.

if nargin < 1 || isempty(doSave)
    doSave = true;
end

model = "source";
subsystem = model + "/PD_only";
load_system(model);

if evalin("base", "exist('base', 'var')") ~= 1
    run(fullfile(fileparts(mfilename("fullpath")), "startup.m"));
end

baseStateZoh = subsystem + "/Zero-Order" + newline + "Hold";
upperController = findOne(subsystem, "BlockType", "MATLABFcn", ...
    "MATLABFcn", "floating_base_lqr_command");

names = [
    "Per-Leg Wrench"
    "Per-Leg Wrench Split"
    "Common Wheel State Input"
    "Common Wheel State"
    "Common Wheel Position LQR"
    "Left QP Input"
    "Right QP Input"
    "Left QP Input ZOH"
    "Right QP Input ZOH"
    "Left QP"
    "Right QP"
    "Left QP Split"
    "Right QP Split"
    "Left Torque Split"
    "Right Torque Split"
    "Left Slack Terminator"
    "Left Feasible Terminator"
    "Left Slack Norm Terminator"
    "Left QP Status Terminator"
    "Left Contact Force Terminator"
    "Right Slack Terminator"
    "Right Feasible Terminator"
    "Right Slack Norm Terminator"
    "Right QP Status Terminator"
    "Right Contact Force Terminator"
    "Coupled QP Input"
    "Coupled QP Input ZOH"
    "Coupled QP"
    "Coupled QP Split"
    "Coupled Slack Terminator"
    "Coupled Feasible Terminator"
    "Coupled Slack Norm Terminator"
    "Coupled QP Status Terminator"
    "Coupled Left Contact Force Terminator"
    "Coupled Right Contact Force Terminator"
    "Symmetry Diagnostics"
    "Symmetry Diagnostics Terminator"
    "NMPC State Split"
    "NMPC Reference Input"
    "NMPC Reference"
    "NMPC Reference Split"
    "NMPC Solver"
    "NMPC Command Mux"
    "NMPC Command Guard"
    "NMPC Guard Split"
    "NMPC Fallback Terminator"
    "NMPC Previous Input"
    "NMPC Fault Terminator"
];
for i = 1:numel(names)
    deleteBlockIfPresent(subsystem + "/" + names(i));
end

set_param(baseStateZoh, "SampleTime", "base.Ts");
set_param(upperController, "OutputDimensions", "3", ...
    "SampleTime", "base.Ts");

wheelStateInput = add_block("simulink/Signal Routing/Mux", ...
    subsystem + "/Common Wheel State Input", ...
    "Inputs", "13", "Position", [720, 485, 725, 705]);
wheelState = add_block(upperController, subsystem + "/Common Wheel State", ...
    "MATLABFcn", "wheel_position_state_signal", ...
    "OutputDimensions", "10", "SampleTime", "base.Ts", ...
    "Position", [780, 565, 955, 605]);
wheelPlanner = add_block(upperController, ...
    subsystem + "/Common Wheel Position LQR", ...
    "MATLABFcn", "wheel_position_lqr_reference", ...
    "OutputDimensions", "4", "SampleTime", "base.Ts", ...
    "Position", [1010, 565, 1195, 605]);

coupledInput = add_block("simulink/Signal Routing/Mux", ...
    subsystem + "/Coupled QP Input", "Inputs", "15", ...
    "Position", [1240, 310, 1245, 880]);
coupledZoh = add_block("simulink/Discrete/Zero-Order Hold", ...
    subsystem + "/Coupled QP Input ZOH", "SampleTime", "base.Ts", ...
    "Position", [1295, 575, 1345, 615]);
coupledQp = add_block(upperController, subsystem + "/Coupled QP", ...
    "MATLABFcn", "coupled_two_leg_qp_signal", "OutputDimensions", "18", ...
    "SampleTime", "base.Ts", "Position", [1395, 575, 1545, 615]);
coupledSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/Coupled QP Split", "Outputs", "[3 3 3 3 1 1 2 2]", ...
    "Position", [1585, 500, 1590, 690]);
leftTorqueSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/Left Torque Split", "Outputs", "3", ...
    "Position", [1640, 475, 1645, 555]);
rightTorqueSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/Right Torque Split", "Outputs", "3", ...
    "Position", [1640, 595, 1645, 675]);

coupledTerminators = addCoupledTerminators(subsystem, 1700, 700);

leftQ = [
    subsystem + "/Fcn1"
    block(subsystem, "PS-Simulink", "Converter13")
    block(subsystem, "PS-Simulink", "Converter16")
];
leftDq = [
    block(subsystem, "PS-Simulink", "Converter17")
    block(subsystem, "PS-Simulink", "Converter14")
    block(subsystem, "PS-Simulink", "Converter15")
];
rightQ = [
    subsystem + "/Fcn"
    block(subsystem, "PS-Simulink", "Converter1")
    block(subsystem, "PS-Simulink", "Converter2")
];
rightDq = [
    block(subsystem, "PS-Simulink", "Converter9")
    block(subsystem, "PS-Simulink", "Converter10")
    block(subsystem, "PS-Simulink", "Converter11")
];

symmetryDiagnostics = add_block("simulink/Signal Routing/Mux", ...
    subsystem + "/Symmetry Diagnostics", "Inputs", "12", ...
    "Position", [1010, 930, 1015, 1110]);
symmetryTerminator = add_block("simulink/Sinks/Terminator", ...
    subsystem + "/Symmetry Diagnostics Terminator", ...
    "Position", [1190, 1005, 1210, 1025]);
connectVectorSources(subsystem, leftQ, symmetryDiagnostics, 1);
connectVectorSources(subsystem, leftDq, symmetryDiagnostics, 4);
connectVectorSources(subsystem, rightQ, symmetryDiagnostics, 7);
connectVectorSources(subsystem, rightDq, symmetryDiagnostics, 10);
connect(subsystem, symmetryDiagnostics, 1, symmetryTerminator, 1);

% Fcn/Fcn1 both convert the Simscape hip angle by pi/2.
connect(subsystem, baseStateZoh, 1, wheelStateInput, 1);
connectVectorSources(subsystem, leftQ, wheelStateInput, 2);
connectVectorSources(subsystem, leftDq, wheelStateInput, 5);
connectVectorSources(subsystem, rightQ, wheelStateInput, 8);
connectVectorSources(subsystem, rightDq, wheelStateInput, 11);
connect(subsystem, wheelStateInput, 1, wheelState, 1);
connect(subsystem, wheelState, 1, wheelPlanner, 1);

wireCoupledInput(subsystem, coupledInput, baseStateZoh, leftQ, leftDq, ...
    rightQ, rightDq, upperController, wheelPlanner);
connect(subsystem, coupledInput, 1, coupledZoh, 1);
connect(subsystem, coupledZoh, 1, coupledQp, 1);
connect(subsystem, coupledQp, 1, coupledSplit, 1);
connect(subsystem, coupledSplit, 1, leftTorqueSplit, 1);
connect(subsystem, coupledSplit, 2, rightTorqueSplit, 1);
for i = 1:numel(coupledTerminators)
    connect(subsystem, coupledSplit, i + 2, coupledTerminators(i), 1);
end

leftActuation = [
    block(subsystem, "Simulink-PS", "Converter5")
    block(subsystem, "Simulink-PS", "Converter6")
    block(subsystem, "Simulink-PS", "Converter7")
];
rightActuation = [
    block(subsystem, "Simulink-PS", "Converter")
    block(subsystem, "Simulink-PS", "Converter1")
    block(subsystem, "Simulink-PS", "Converter4")
];
for i = 1:3
    connect(subsystem, leftTorqueSplit, i, leftActuation(i), 1);
    connect(subsystem, rightTorqueSplit, i, rightActuation(i), 1);
end

setZeroPulseAmplitude(model);
logOutput(coupledQp, "coupledQpSignal");
logOutput(wheelState, "commonWheelStateSignal");
logOutput(wheelPlanner, "commonWheelReference");
logOutput(upperController, "totalUpperCommand");
logOutput(symmetryDiagnostics, "symmetryLegState");

set_param(model, "SolverType", "Variable-step", "Solver", "ode15s", ...
    "MaxStep", "base.Ts", "RelTol", "1e-3", "AbsTol", "1e-4");
set_param(model, "SimulationCommand", "update");
if doSave
    save_system(model, [], "OverwriteIfChangedOnDisk", true);
end
fprintf("Configured coupled two-leg floating-base LQR-QP control in %s.\n", model);
end

function wireCoupledInput(parent, inputMux, baseState, qLeft, dqLeft, ...
        qRight, dqRight, wrench, wheelReference)
connect(parent, baseState, 1, inputMux, 1);
connectVectorSources(parent, qLeft, inputMux, 2);
connectVectorSources(parent, dqLeft, inputMux, 5);
connectVectorSources(parent, qRight, inputMux, 8);
connectVectorSources(parent, dqRight, inputMux, 11);
connect(parent, wrench, 1, inputMux, 14);
connect(parent, wheelReference, 1, inputMux, 15);
end

function connectVectorSources(parent, sources, destination, firstPort)
for i = 1:numel(sources)
    connect(parent, sources(i), 1, destination, firstPort + i - 1);
end
end

function paths = addCoupledTerminators(parent, x, y)
suffixes = ["Slack", "Feasible", "Slack Norm", "QP Status", ...
    "Left Contact Force", "Right Contact Force"];
paths = strings(numel(suffixes), 1);
for i = 1:numel(suffixes)
    paths(i) = parent + "/Coupled " + suffixes(i) + " Terminator";
    add_block("simulink/Sinks/Terminator", paths(i), ...
        "Position", [x, y + 35*(i - 1), x + 20, y + 20 + 35*(i - 1)]);
end
end

function value = block(parent, prefix, suffix)
value = parent + "/" + prefix + newline + suffix;
end

function blockPath = findOne(parent, varargin)
matches = find_system(parent, "SearchDepth", 1, varargin{:});
matches = setdiff(string(matches), string(parent), "stable");
if numel(matches) ~= 1
    error("configure_symmetric_two_leg_simulink:BlockLookup", ...
        "Expected one matching block under %s, found %d.", parent, numel(matches));
end
blockPath = matches(1);
end

function deleteBlockIfPresent(blockPath)
if getSimulinkBlockHandle(blockPath) ~= -1
    delete_block(blockPath);
end
end

function connect(parent, source, sourcePort, destination, destinationPort)
sourceHandles = get_param(source, "PortHandles");
destinationHandles = get_param(destination, "PortHandles");
disconnectPort(destinationHandles.Inport(destinationPort));
add_line(parent, sourceHandles.Outport(sourcePort), ...
    destinationHandles.Inport(destinationPort), "autorouting", "on");
end

function disconnectPort(portHandle)
line = get_param(portHandle, "Line");
if isnumeric(line) && isscalar(line) && line ~= -1 && line ~= 0
    delete_line(line);
end
end

function setZeroPulseAmplitude(model)
blocks = find_system(model, "LookUnderMasks", "all", "FollowLinks", "on", ...
    "BlockType", "DiscretePulseGenerator");
for i = 1:numel(blocks)
    set_param(blocks{i}, "Amplitude", "0");
end
end

function logOutput(blockPath, name)
handles = get_param(blockPath, "PortHandles");
line = get_param(handles.Outport(1), "Line");
if line ~= -1
    set_param(line, "Name", name);
end
set_param(handles.Outport(1), "DataLogging", "on", ...
    "DataLoggingNameMode", "Custom", "DataLoggingName", name);
end
