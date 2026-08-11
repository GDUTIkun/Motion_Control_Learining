function test_full_base_nmpc()
%TEST_FULL_BASE_NMPC Validate the 16-state, per-side-wrench NMPC.

run(fullfile(fileparts(mfilename("fullpath")), "startup.m"));
config = fullBaseNmpc;
model = config.model;

assert(isequal(size(model.A), [16, 16]));
assert(isequal(size(model.B), [16, 12]));
assert(norm(fullStateDerivative(model.xEq, model.uEq, model, leg), inf) ...
    < 1e-12, "The full NMPC equilibrium is inconsistent.");
assert(all(config.uMin < config.uMax | config.uMin == config.uMax));

planner = [wheelLqr.neutral; 0; 0; wheelLqr.neutral];
reference = full_base_nmpc_reference( ...
    [0; planner; model.uEq], baseLqr, config, wheelLqr);
assert(numel(reference) == config.referenceSize);
assert(norm(reference(1:16) - model.xEq, inf) < 1e-12);
assert(norm(reference(17:28) - model.uEq, inf) < 1e-12);

clear full_base_nmpc_command
command = full_base_nmpc_command([0; model.uEq; 0; 0], config);
assert(numel(command) == 13 && command(end) == 0);
assert(norm(command(1:12) - model.uEq, inf) < 1e-12);

stateInput = [0; base.x0; zeros(4, 1); zeros(2, 1); ...
    leg.q0; leg.dq0; leg.q0; leg.dq0];
stateSignal = full_base_nmpc_state_signal(stateInput, base, leg, ctrl);
assert(numel(stateSignal) == 18 && all(isfinite(stateSignal)));
assert(abs(stateSignal(14) - stateSignal(15)) < 1e-12);

simulateDir = fileparts(mfilename("fullpath"));
repoRoot = fileparts(fileparts(fileparts(simulateDir)));
addpath(fullfile(repoRoot, "tools", "casadi"));
addpath(fullfile(repoRoot, "tools", "acados", "interfaces", ...
    "acados_matlab_octave"));
acados_env_variables_windows;
ocp = full_base_nmpc_ocp(base, leg, config);
ocp.make_consistent();
assert(ocp.dims.nx == 16 && ocp.dims.nu == 12);
assert(ocp.dims.ny == 40 && ocp.dims.nh == 4);

fprintf("Full 8-DoF base NMPC checks passed.\n");
end

function dX = fullStateDerivative(x, u, model, leg)
FL = u(1:3);
TL = u(4:6);
FR = u(7:9);
TR = u(10:12);
force = FL + FR;
d = model.halfTrack;
h = model.rWzEq;
rollMoment = d*(FL(3) - FR(3)) - h*(FL(2) + FR(2)) ...
    + TL(1) + TR(1);
pitchMoment = (x(13) - model.xiEq)*FL(3) ...
    + (x(14) - model.xiEq)*FR(3) - h*(FL(1) + FR(1)) ...
    + TL(2) + TR(2);
yawMoment = -d*FL(1) + d*FR(1) ...
    + x(13)*FL(2) + x(14)*FR(2) + TL(3) + TR(3);
D = model.rollingDenominator;
dX = [x(7:12); force(1:2)/model.m; force(3)/model.m - 9.81; ...
    rollMoment/model.inertia(1); pitchMoment/model.inertia(2); ...
    yawMoment/model.inertia(3); x(15:16); ...
    -force(1)/model.m - (leg.r*FL(1) + TL(2))/D; ...
    -force(1)/model.m - (leg.r*FR(1) + TR(2))/D];
end
