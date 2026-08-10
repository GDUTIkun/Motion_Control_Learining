function test_base_nmpc()
%TEST_BASE_NMPC Minimal formulation, reference, and direct-command checks.

run(fullfile(fileparts(mfilename("fullpath")), "startup.m"));

assert(isscalar(baseNmpc.N) && baseNmpc.N >= 1 ...
    && baseNmpc.N == fix(baseNmpc.N));
assert(isscalar(baseNmpc.Ts) && isfinite(baseNmpc.Ts) && baseNmpc.Ts > 0);
assert(all(eig((baseNmpc.Q + baseNmpc.Q') / 2) > 0));
assert(all(eig((baseNmpc.R1 + baseNmpc.R1') / 2) > 0));
assert(all(eig((baseNmpc.R2 + baseNmpc.R2') / 2) > 0));
assert(all(eig((baseNmpc.W_e + baseNmpc.W_e') / 2) > 0));
assert(all(baseNmpc.uMin < baseNmpc.uMax));

dX = nonlinearState(baseNmpc.model.xEq, baseNmpc.model.uEq, ...
    baseNmpc.model, leg);
assert(norm(dX, inf) < 1e-12, "The NMPC equilibrium is inconsistent.");
assert(isequal(size(baseNmpc.model.A), [8, 8]) ...
    && isequal(size(baseNmpc.model.B), [8, 3]));
assert(abs(baseNmpc.model.A(6, 7) ...
    - baseNmpc.model.uEq(2)/base.Iyy) < 1e-12);
assert(abs(baseNmpc.model.B(6, 1) ...
    + baseNmpc.model.rWzEq/base.Iyy) < 1e-12);

reference = base_nmpc_reference(0, baseLqr, baseNmpc);
assert(numel(reference) == baseNmpc.referenceSize);
assert(all(isfinite(reference)));
assert(norm(reference(1:6) - floating_base_reference(0, baseLqr), inf) < 1e-12);
assert(norm(reference(7:8) - [wheelLqr.neutral; 0], inf) < 1e-12);
assert(norm(reference(12:14) - baseNmpc.model.uEq, inf) < 1e-12);
pathReference = reshape(reference(15:end-8), 14, baseNmpc.N - 1);
assert(norm(pathReference(1:6, 1) - ...
    floating_base_reference(baseNmpc.Ts, baseLqr), inf) < 1e-12);
assert(norm(reference(end-7:end-2) - ...
    floating_base_reference(baseNmpc.N*baseNmpc.Ts, baseLqr), inf) < 1e-12);

trajectory = baseLqr.trajectory;
roundTripEnd = trajectory.settleTime + 2*(trajectory.accelDuration ...
    + trajectory.cruiseDuration + trajectory.decelDuration) ...
    + trajectory.turnHoldDuration;
startReference = floating_base_reference(0, baseLqr);
endReference = floating_base_reference(roundTripEnd, baseLqr);
assert(norm(startReference - endReference, inf) < 1e-12, ...
    "The configured round-trip reference does not close continuously.");

uBody = baseNmpc.model.uEq;
expectedCommand = [-uBody(1:2); uBody(3)];
clear base_nmpc_command
y = base_nmpc_command([0; uBody; 0; 0], baseNmpc);
assert(norm(y(1:3) - expectedCommand, inf) < 1e-12 && y(4) == 0);

failed = base_nmpc_command([baseNmpc.Ts; nan(3, 1); 1; 0], baseNmpc);
assert(norm(failed(1:3) - expectedCommand, inf) < 1e-12 && failed(4) == 1);

timedOutConfig = baseNmpc;
timedOutConfig.maxSolveTime = 0;
timedOut = base_nmpc_command([2*baseNmpc.Ts; uBody; 0; eps], timedOutConfig);
assert(timedOut(4) == 1);

simulateDir = fileparts(mfilename("fullpath"));
repoRoot = fileparts(fileparts(fileparts(simulateDir)));
addpath(fullfile(repoRoot, "tools", "casadi"));
addpath(fullfile(repoRoot, "tools", "acados", "interfaces", ...
    "acados_matlab_octave"));
acados_env_variables_windows;
ocp = base_nmpc_ocp(base, leg, baseNmpc);
ocp.make_consistent();
assert(ocp.dims.nx == 8 && ocp.dims.nu == 3);
assert(ocp.dims.ny == 14 && ocp.dims.nh == 2);
assert(ocp.solver_options.N_horizon == baseNmpc.N);

fprintf("Base NMPC checks passed.\n");
end

function dX = nonlinearState(x, u, model, leg)
D = leg.mw*leg.r + leg.Iw/leg.r;
dX = [x(4:6); u(1)/model.m; u(2)/model.m - model.g; ...
    ((x(7)-model.xiEq)*u(2) - model.rWzEq*u(1) + u(3))/model.Iyy; x(8); ...
    -u(1)/model.m - (u(1)*leg.r + u(3))/(2*D)];
end
