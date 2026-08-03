function y = floating_base_lqr_force(x, baseLqr)
%FLOATING_BASE_LQR_FORCE Simulink-friendly upper-layer LQR force command.
%
% Preferred input:
%   x = [t; xB; zB; thetaB; dxB; dzB; dthetaB]
%
% Compatibility input used by the existing hip-force demo:
%   x = [t; xH; zH; dxH; dzH]
% In this mode thetaB and dthetaB are assumed zero.
%
% Output:
%   y = [xRef; zRef; FHx_ext; FHz_ext]
%
% FH_ext is the force applied by the body to the leg. It is the negative of
% the LQR body force command because the leg dynamics use the external load
% acting on the leg at the hip interface.

if nargin < 2 || isempty(baseLqr)
    baseLqr = evalin("base", "baseLqr");
end

x = double(x(:));
if numel(x) == 7
    X = x(2:7);
elseif numel(x) == 6
    X = x(:);
elseif numel(x) == 5
    X = [x(2); x(3); 0; x(4); x(5); 0];
else
    error("floating_base_lqr_force:InvalidInput", ...
        "Expected x = [t;x;z;theta;dx;dz;dtheta] or [t;x;z;dx;dz].");
end

xRef = baseLqr.xRef(:);
uBody = baseLqr.model.uEq - baseLqr.K * (X - xRef);

forceMax = baseLqr.forceMax(:);
if isscalar(forceMax)
    forceMax = repmat(forceMax, 2, 1);
end
uBody(1:2) = min(max(uBody(1:2), -forceMax), forceMax);
uBody(3) = min(max(uBody(3), -baseLqr.momentMax), baseLqr.momentMax);

FH_ext = -uBody(1:2);
y = [xRef(1); xRef(2); FH_ext];
end
