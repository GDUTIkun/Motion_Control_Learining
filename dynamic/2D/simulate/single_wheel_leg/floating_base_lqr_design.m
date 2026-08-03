function baseLqr = floating_base_lqr_design(base, Q, R)
%FLOATING_BASE_LQR_DESIGN Build A/B matrices and continuous-time LQR gain.
%
% Returns a struct containing:
%   model: floating-base state-space data
%   K:     u = uEq - K*(x - xRef)
%   Q,R:   weights used for LQR

if nargin < 1 || isempty(base)
    base = evalin("base", "base");
end

model = floating_base_state_space(base);

if nargin < 2 || isempty(Q)
    Q = getFieldOrDefault(base, "Q", diag([25, 80, 120, 8, 16, 10]));
end
if nargin < 3 || isempty(R)
    R = getFieldOrDefault(base, "R", diag([1/80^2, 1/140^2, 1/60^2]));
end

Q = double(Q);
R = double(R);

[K, S, poles] = localLqr(model.A, model.B, Q, R);

baseLqr = struct();
baseLqr.model = model;
baseLqr.K = K;
baseLqr.S = S;
baseLqr.poles = poles;
baseLqr.Q = Q;
baseLqr.R = R;
baseLqr.forceMax = getFieldOrDefault(base, "forceMax", [inf; inf]);
baseLqr.momentMax = getFieldOrDefault(base, "momentMax", inf);
baseLqr.xRef = getFieldOrDefault(base, "xRef", model.xEq);
baseLqr.xRef = baseLqr.xRef(:);
end

function [K, S, poles] = localLqr(A, B, Q, R)
if exist("lqr", "file") == 2
    [K, S, poles] = lqr(A, B, Q, R);
    return;
end

if exist("care", "file") == 2
    [S, ~, poles] = care(A, B, Q, R);
    K = R \ (B' * S);
    return;
end

if exist("icare", "file") == 2
    [S, ~, poles] = icare(A, B, Q, R);
    K = R \ (B' * S);
    return;
end

% Last-resort continuous-time algebraic Riccati solve through the
% Hamiltonian invariant subspace. This keeps the demo runnable on MATLAB
% installations without Control System Toolbox.
n = size(A, 1);
G = B * (R \ B');
H = [A, -G; -Q, -A'];
[V, D] = eig(H);
stableIdx = find(real(diag(D)) < 0);
if numel(stableIdx) ~= n
    error("floating_base_lqr_design:MissingStableSubspace", ...
        "Could not isolate the stable Hamiltonian subspace.");
end
Vstable = V(:, stableIdx);
V1 = Vstable(1:n, :);
V2 = Vstable(n+1:end, :);
S = real(V2 / V1);
S = 0.5 * (S + S');
K = R \ (B' * S);
poles = eig(A - B*K);
end

function value = getFieldOrDefault(s, name, defaultValue)
if isfield(s, name)
    value = s.(name);
else
    value = defaultValue;
end
end
