function y = hip_position_pd(x, hip)
%HIP_POSITION_PD Upper-layer surrogate that outputs desired hip force.
%
% Input:
%   x = [t; xH; yH; dxH; dyH]
%
% Output:
%   y = [xH_ref; yH_ref; FHx_ext; FHy_ext]
%
% F_H_des is the desired force that the leg applies to the hip/body. The leg
% model sees the opposite external force at the hip interface:
%   F_H_ext = -F_H_des

if nargin < 2 || isempty(hip)
    hip = evalin("base", "hip");
end

x = double(x(:));
if numel(x) ~= 5
    error("hip_position_pd:InvalidInput", ...
        "Expected x = [t; xH; yH; dxH; dyH].");
end

t = x(1);
pH = x(2:3);
dpH = x(4:5);

[pRef, dpRef, ddpRef] = hip_reference_trajectory(t, hip);
virtualMass = hip.virtualMass(:);
if isscalar(virtualMass)
    virtualMass = repmat(virtualMass, 2, 1);
end

FH_des = virtualMass .* ddpRef ...
    + hip.Kd * (dpRef - dpH) ...
    + hip.Kp * (pRef - pH) ...
    + hip.forceBias(:);

FH_des = min(max(FH_des, -hip.forceMax(:)), hip.forceMax(:));
FH_ext = -FH_des;

y = [pRef; FH_ext];
end
