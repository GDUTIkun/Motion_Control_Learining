function y = floating_base_lqr_command(x, baseLqr)
%FLOATING_BASE_LQR_COMMAND Simulink upper-layer command for base-state LQR.
%
% Input:
%   x = [t; xB; zB; thetaB; dxB; dzB; dthetaB]
% or:
%   x = [xB; zB; thetaB; dxB; dzB; dthetaB]
%
% Output:
%   y = [FHx_ext; FHz_ext; MBy_des]
%
% The LQR state uses the floating-base CoM state. The LQR wrench
% [FHx_des; FHz_des; MBy_des] is the wrench applied by the leg to the body.
% The leg QP uses the external force applied by the body to the leg, so this
% function flips only the force components:
%   FH_ext = -[FHx_des; FHz_des]
%   MBy_des is kept as the body-side desired pure pitch moment.

if nargin < 2 || isempty(baseLqr)
    baseLqr = evalin("base", "baseLqr");
end

uBody = floating_base_lqr_wrench(x, baseLqr);
FH_ext = -uBody(1:2);
MBy_des = uBody(3);

y = [FH_ext; MBy_des];
end
