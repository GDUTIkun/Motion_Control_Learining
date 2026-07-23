function y = reference_output(t)
%REFERENCE_OUTPUT Output reference trajectory for Simulink Scope.
%
% Input:
%   t: simulation time
%
% Output:
%   y = [qh_d; qk_d; dqh_d; dqk_d; ddqh_d; ddqk_d]

traj = evalin("base", "traj");

[qd, dqd, ddqd] = two_link_leg_reference(t, traj);

y = [qd; dqd; ddqd];
end