function reference = base_nmpc_reference(t, baseLqr, baseNmpc)
%BASE_NMPC_REFERENCE Return flattened S-Function references over the horizon.
%
% Layout:
%   [y_ref_0(9); y_ref(stages 1:N-1, 9 each); y_ref_e(6)]

if nargin < 2 || isempty(baseLqr)
    baseLqr = evalin("base", "baseLqr");
end
if nargin < 3 || isempty(baseNmpc)
    baseNmpc = evalin("base", "baseNmpc");
end

t = double(t(1));
N = baseNmpc.N;
pathReference = zeros(9, max(N - 1, 0));

for k = 0:N
    [xRef, aRef] = floating_base_reference(t + k*baseNmpc.Ts, baseLqr);
    uRef = feedforwardWrench(aRef, baseLqr.model);
    uRef = min(max(uRef, baseNmpc.uMin(:)), baseNmpc.uMax(:));

    if k == 0
        initialReference = [xRef; uRef];
    elseif k < N
        pathReference(:, k) = [xRef; uRef];
    else
        terminalReference = xRef;
    end
end

reference = [initialReference; pathReference(:); terminalReference];
end

function u = feedforwardWrench(aRef, model)
FHx = model.m * aRef(1);
FHz = model.m * (model.g + aRef(2));
MBy = model.Iyy * aRef(3) ...
    - model.rHEq(1) * FHz + model.rHEq(2) * FHx;
u = [FHx; FHz; MBy];
end
