function [qd, dqd, ddqd] = two_link_leg_reference(t, traj)
%TWO_LINK_LEG_REFERENCE Smooth joint-space reference trajectory.
%
% qd follows the startup convention:
%   qh_d = offset_h + amplitude_h*sin(omega*t + phase)
%   qk_d = offset_k + amplitude_k*sin(omega*t + phase)

if nargin < 2 || isempty(traj)
    traj = evalin("base", "traj");
end

t = double(t);
phase = traj.omega * t + traj.phase;

qd = traj.offset + traj.amplitude * sin(phase);
dqd = traj.amplitude * traj.omega * cos(phase);
ddqd = -traj.amplitude * traj.omega^2 * sin(phase);

qd = qd(:);
dqd = dqd(:);
ddqd = ddqd(:);
end
