function y = base_nmpc_command(x, baseNmpc)
%BASE_NMPC_COMMAND Validate NMPC output and select the lower-QP command.
%
% x = [NMPC body wrench(3); status; CPU time; LQR QP command(3)]
% y = [selected QP command(3); fallback active]

if nargin < 2 || isempty(baseNmpc)
    baseNmpc = evalin("base", "baseNmpc");
end

x = double(x(:));
if numel(x) ~= 8
    error("base_nmpc_command:InvalidInput", "Expected an 8-element input.");
end

uBody = x(1:3);
status = x(4);
cpuTime = x(5);
lqrCommand = x(6:8);
valid = baseNmpc.enabled && status == 0 ...
    && all(isfinite(uBody)) && isfinite(cpuTime) ...
    && cpuTime <= baseNmpc.maxSolveTime;

if valid
    command = [-uBody(1:2); uBody(3)];
    fallback = 0;
else
    command = lqrCommand;
    fallback = 1;
end

y = [command; fallback];
end
