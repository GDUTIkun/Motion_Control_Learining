function test_base_nmpc()
%TEST_BASE_NMPC Minimal formulation, reference, and fallback checks.

run(fullfile(fileparts(mfilename("fullpath")), "startup.m"));

assert(isscalar(baseNmpc.N) && baseNmpc.N >= 1 ...
    && baseNmpc.N == fix(baseNmpc.N));
assert(isscalar(baseNmpc.Ts) && isfinite(baseNmpc.Ts) && baseNmpc.Ts > 0);
assert(all(eig((baseNmpc.Q + baseNmpc.Q') / 2) > 0));
assert(all(eig((baseNmpc.R + baseNmpc.R') / 2) > 0));
assert(all(eig((baseNmpc.W_e + baseNmpc.W_e') / 2) > 0));
assert(all(baseNmpc.uMin < baseNmpc.uMax));

dBase = floating_base_nonlinear_state(0, base.xEq, ...
    baseLqr.model.uEq, base);
dX = [dBase; 0; 0];
assert(norm(dX, inf) < 1e-12, "The NMPC equilibrium is inconsistent.");
assert(isequal(size(baseNmpc.model.A), [8, 8]) ...
    && isequal(size(baseNmpc.model.B), [8, 3]));

reference = base_nmpc_reference(0, baseLqr, baseNmpc);
assert(numel(reference) == baseNmpc.referenceSize);
assert(all(isfinite(reference)));
assert(norm(reference(1:6) - floating_base_reference(0, baseLqr), inf) < 1e-12);
assert(norm(reference(7:8) - [wheelLqr.neutral; 0], inf) < 1e-12);
pathReference = reshape(reference(12:end-8), 11, baseNmpc.N - 1);
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

uBody = baseLqr.model.uEq;
lqrCommand = [-uBody(1:2); uBody(3)];
y = base_nmpc_command([uBody; 0; 0; zeros(3, 1)], baseNmpc);
expectedCommand = baseNmpc.commandBlend * lqrCommand;
assert(norm(y(1:3) - expectedCommand, inf) < 1e-12 && y(4) == 0);

failed = base_nmpc_command([uBody; 1; 0; lqrCommand], baseNmpc);
assert(norm(failed(1:3) - lqrCommand, inf) < 1e-12 && failed(4) == 1);

timedOutConfig = baseNmpc;
timedOutConfig.maxSolveTime = 0;
timedOut = base_nmpc_command([uBody; 0; eps; lqrCommand], timedOutConfig);
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
assert(ocp.solver_options.N_horizon == baseNmpc.N);

fprintf("Base NMPC checks passed.\n");
end
