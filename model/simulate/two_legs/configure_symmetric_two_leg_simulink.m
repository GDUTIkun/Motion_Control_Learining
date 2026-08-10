function configure_symmetric_two_leg_simulink(doSave)
%CONFIGURE_SYMMETRIC_TWO_LEG_SIMULINK Wire the common-mode LQR to two leg QPs.
%
% The upper controller produces the total [Fx; Fz; My] body wrench. A 0.5
% gain sends the same symmetric share to each independent 3-DoF leg QP.
% Left/right wheel states are averaged only for the common wheel-position
% planner; no differential state or differential controller is introduced.

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
];
for i = 1:numel(names)
    deleteBlockIfPresent(subsystem + "/" + names(i));
end

set_param(baseStateZoh, "SampleTime", "base.Ts");
set_param(upperController, "OutputDimensions", "3", ...
    "SampleTime", "base.Ts");

perLegWrench = add_block("simulink/Math Operations/Gain", ...
    subsystem + "/Per-Leg Wrench", ...
    "Gain", "base.symmetricLoadShare", ...
    "Multiplication", "Element-wise(K.*u)", ...
    "Position", [520, 370, 610, 405]);
perLegWrenchSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/Per-Leg Wrench Split", ...
    "Outputs", "3", "Position", [655, 350, 660, 430]);

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

leftInput = add_block("simulink/Signal Routing/Mux", ...
    subsystem + "/Left QP Input", "Inputs", "11", ...
    "Position", [1240, 310, 1245, 500]);
rightInput = add_block("simulink/Signal Routing/Mux", ...
    subsystem + "/Right QP Input", "Inputs", "11", ...
    "Position", [1240, 690, 1245, 880]);
leftZoh = add_block("simulink/Discrete/Zero-Order Hold", ...
    subsystem + "/Left QP Input ZOH", "SampleTime", "base.Ts", ...
    "Position", [1295, 385, 1345, 425]);
rightZoh = add_block("simulink/Discrete/Zero-Order Hold", ...
    subsystem + "/Right QP Input ZOH", "SampleTime", "base.Ts", ...
    "Position", [1295, 765, 1345, 805]);
leftQp = add_block(upperController, subsystem + "/Left QP", ...
    "MATLABFcn", "controller_qp_signal", "OutputDimensions", "13", ...
    "SampleTime", "base.Ts", "Position", [1395, 385, 1535, 425]);
rightQp = add_block(upperController, subsystem + "/Right QP", ...
    "MATLABFcn", "controller_qp_signal", "OutputDimensions", "13", ...
    "SampleTime", "base.Ts", "Position", [1395, 765, 1535, 805]);

leftSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/Left QP Split", "Outputs", "[3 3 3 1 1 2]", ...
    "Position", [1585, 350, 1590, 465]);
rightSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/Right QP Split", "Outputs", "[3 3 3 1 1 2]", ...
    "Position", [1585, 730, 1590, 845]);
leftTorqueSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/Left Torque Split", "Outputs", "3", ...
    "Position", [1640, 335, 1645, 415]);
rightTorqueSplit = add_block("simulink/Signal Routing/Demux", ...
    subsystem + "/Right Torque Split", "Outputs", "3", ...
    "Position", [1640, 715, 1645, 795]);

leftTerminators = addTerminators(subsystem, "Left", 1640, 440);
rightTerminators = addTerminators(subsystem, "Right", 1640, 820);

connect(subsystem, upperController, 1, perLegWrench, 1);
connect(subsystem, perLegWrench, 1, perLegWrenchSplit, 1);

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

wireLegInput(subsystem, leftInput, baseStateZoh, leftQ, leftDq, ...
    perLegWrenchSplit, wheelPlanner);
wireLegInput(subsystem, rightInput, baseStateZoh, rightQ, rightDq, ...
    perLegWrenchSplit, wheelPlanner);
connect(subsystem, leftInput, 1, leftZoh, 1);
connect(subsystem, rightInput, 1, rightZoh, 1);
connect(subsystem, leftZoh, 1, leftQp, 1);
connect(subsystem, rightZoh, 1, rightQp, 1);
connect(subsystem, leftQp, 1, leftSplit, 1);
connect(subsystem, rightQp, 1, rightSplit, 1);
connect(subsystem, leftSplit, 1, leftTorqueSplit, 1);
connect(subsystem, rightSplit, 1, rightTorqueSplit, 1);
for i = 1:numel(leftTerminators)
    connect(subsystem, leftSplit, i + 1, leftTerminators(i), 1);
    connect(subsystem, rightSplit, i + 1, rightTerminators(i), 1);
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
logOutput(leftQp, "leftQpSignal");
logOutput(rightQp, "rightQpSignal");
logOutput(wheelState, "commonWheelStateSignal");
logOutput(wheelPlanner, "commonWheelReference");
logOutput(perLegWrench, "perLegWrenchCommand");
logOutput(symmetryDiagnostics, "symmetryLegState");

set_param(model, "SolverType", "Variable-step", "Solver", "ode15s", ...
    "MaxStep", "base.Ts", "RelTol", "1e-3", "AbsTol", "1e-4");
set_param(model, "SimulationCommand", "update");
if doSave
    save_system(model, [], "OverwriteIfChangedOnDisk", true);
end
fprintf("Configured symmetric two-leg LQR-QP control in %s.\n", model);
end

function wireLegInput(parent, inputMux, baseState, q, dq, wrench, wheelReference)
connect(parent, baseState, 1, inputMux, 1);
connectVectorSources(parent, q, inputMux, 2);
connectVectorSources(parent, dq, inputMux, 5);
for i = 1:3
    connect(parent, wrench, i, inputMux, i + 7);
end
connect(parent, wheelReference, 1, inputMux, 11);
end

function connectVectorSources(parent, sources, destination, firstPort)
for i = 1:numel(sources)
    connect(parent, sources(i), 1, destination, firstPort + i - 1);
end
end

function paths = addTerminators(parent, side, x, y)
suffixes = ["Slack", "Feasible", "Slack Norm", "QP Status", ...
    "Contact Force"];
paths = strings(numel(suffixes), 1);
for i = 1:numel(suffixes)
    paths(i) = parent + "/" + side + " " + suffixes(i) + " Terminator";
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
