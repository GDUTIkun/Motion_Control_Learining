function y = floating_base_lqr_wrench(x, baseLqr)
%FLOATING_BASE_LQR_WRENCH Return full upper-layer LQR body wrench.
%
% Output:
%   y = [FHx_des; FHz_des; MBy_des]
%
% The returned wrench is applied by the leg to the body. The leg QP should
% receive FH_ext = -[FHx_des; FHz_des] and optional MBy_des.

if nargin < 2 || isempty(baseLqr)
    baseLqr = evalin("base", "baseLqr");
end

x = double(x(:));
if numel(x) == 7
    X = x(2:7);
elseif numel(x) == 6
    X = x(:);
else
    error("floating_base_lqr_wrench:InvalidInput", ...
        "Expected x = [t;x;z;theta;dx;dz;dtheta] or 6-state vector.");
end

uBody = baseLqr.model.uEq - baseLqr.K * (X - baseLqr.xRef(:));

forceMax = baseLqr.forceMax(:);
if isscalar(forceMax)
    forceMax = repmat(forceMax, 2, 1);
end
uBody(1:2) = min(max(uBody(1:2), -forceMax), forceMax);
uBody(3) = min(max(uBody(3), -baseLqr.momentMax), baseLqr.momentMax);

y = uBody(:);
end
