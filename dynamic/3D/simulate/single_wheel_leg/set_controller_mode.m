function set_controller_mode(mode)
%SET_CONTROLLER_MODE Switch the top-level controller used by source.slx.
%
% Usage:
%   set_controller_mode("nominal")     % existing controller.m
%   set_controller_mode("constraint")  % controller_constraint.m

if nargin < 1 || strlength(string(mode)) == 0
    mode = "constraint";
end

mode = lower(string(mode));
switch mode
    case {"nominal", "default", "ignore_fc"}
        controllerFcn = "controller";
    case {"constraint", "constrained", "contact"}
        controllerFcn = "controller_constraint";
    case {"qp", "wbc", "qp_constraint"}
        controllerFcn = "controller_qp";
    otherwise
        error("set_controller_mode:InvalidMode", ...
            "Use mode = 'nominal' or 'constraint'.");
end

model = "source";
load_system(model);
blocks = find_system(model, "LookUnderMasks", "all", ...
    "FollowLinks", "on", "BlockType", "MATLABFcn", ...
    "MATLABFcn", "controller");
blocks = [blocks; find_system(model, "LookUnderMasks", "all", ...
    "FollowLinks", "on", "BlockType", "MATLABFcn", ...
    "MATLABFcn", "controller_constraint")];
blocks = [blocks; find_system(model, "LookUnderMasks", "all", ...
    "FollowLinks", "on", "BlockType", "MATLABFcn", ...
    "MATLABFcn", "controller_qp")];

if isempty(blocks)
    error("set_controller_mode:MissingControllerBlock", ...
        "Could not find the top-level controller block.");
end

set_param(blocks{1}, "MATLABFcn", controllerFcn, ...
    "OutputDimensions", "3", "Output1D", "on");
save_system(model);
fprintf("Set %s top-level controller to %s.\n", model, controllerFcn);
end
