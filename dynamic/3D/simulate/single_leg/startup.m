% Startup values for a fixed-hip planar two-link leg tracking demo.
%
% Coordinate convention:
%   - hip is the origin
%   - +x points right, +z points up
%   - qh = 0 and qk = 0 put both links vertically downward
%   - positive qh swings the thigh toward +x
%   - positive qk bends the shank further toward +x relative to the thigh

thisFile = mfilename("fullpath");
simulateDir = fileparts(thisFile);
addpath(simulateDir);

leg = struct();
leg.L1 = 0.35;
leg.L2 = 0.35;
leg.c1 = leg.L1 / 2;
leg.c2 = leg.L2 / 2;
leg.m1 = 1.20;
leg.m2 = 0.80;
leg.width = 0.04;
leg.depth = 0.04;
leg.g = 9.81;

% Uniform rectangular rods, inertia about the out-of-plane joint axis.
leg.I1 = leg.m1 * (leg.L1^2 + leg.width^2) / 12;
leg.I2 = leg.m2 * (leg.L2^2 + leg.width^2) / 12;

traj = struct();
traj.freq = 0.25;
traj.omega = 2 * pi * traj.freq;
traj.phase = -pi / 2;
traj.offset = deg2rad([15; 50]);
traj.amplitude = deg2rad([8; 12]);

[leg.q0, leg.dq0, leg.ddq0] = two_link_leg_reference(0, traj);

ctrl = struct();
ctrl.bandwidthHz = [4.98501803244907; 4.98005206851897];
ctrl.wn = 2 * pi * ctrl.bandwidthHz;
ctrl.zeta = [1.95160913237089; 0.941918197483681];

% Computed-torque gains. These are acceleration-level gains that make the
% ideal tracking error obey: e_ddot + Kd*e_dot + Kp*e = 0.
ctrl.Kp = diag(ctrl.wn.^2);
ctrl.Kd = diag(2 * ctrl.zeta .* ctrl.wn);

% Pure-PD comparison gains are torque-level gains, estimated from the
% initial-configuration effective inertia.
M0 = two_link_leg_dynamics(leg.q0, leg.dq0, leg, "M");
M_eff = diag(M0);
ctrl.pdKp = diag(M_eff .* ctrl.wn.^2);
ctrl.pdKd = diag(2 * ctrl.zeta .* M_eff .* ctrl.wn);
ctrl.tauMax = [45; 45];

leg.M = @(q) two_link_leg_dynamics(q, [0; 0], leg, "M");
leg.C = @(q, dq) two_link_leg_dynamics(q, dq, leg, "C");
leg.G = @(q) two_link_leg_dynamics(q, [0; 0], leg, "G");
leg.dynamics = @(q, dq) two_link_leg_dynamics(q, dq, leg);
traj.reference = @(t) two_link_leg_reference(t, traj);

assignin("base", "leg", leg);
assignin("base", "ctrl", ctrl);
assignin("base", "traj", traj);

fprintf("Loaded planar two-link leg parameters into base workspace.\n");
fprintf("Initial q0 = [%.4f; %.4f] rad, dq0 = [%.4f; %.4f] rad/s.\n", ...
    leg.q0(1), leg.q0(2), leg.dq0(1), leg.dq0(2));
fprintf("Computed-torque gains: wn = [%.3f; %.3f] rad/s, zeta = [%.2f; %.2f].\n", ...
    ctrl.wn(1), ctrl.wn(2), ctrl.zeta(1), ctrl.zeta(2));
