function set_controller_mode(mode)
%SET_CONTROLLER_MODE Switch the top-level controller used by source.slx.
%
% Usage:
%   set_controller_mode()      % controller_qp.m
%   set_controller_mode("qp")  % controller_qp.m

if nargin < 1 || strlength(string(mode)) == 0
    mode = "qp";
end

mode = lower(string(mode));
switch mode
    case {"qp", "wbc", "qp_constraint", "default", "nominal", ...
            "constraint", "constrained", "contact", "ignore_fc"}
        controllerFcn = "controller_qp";
    otherwise
        error("set_controller_mode:InvalidMode", ...
            "Use mode = 'qp'.");
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
