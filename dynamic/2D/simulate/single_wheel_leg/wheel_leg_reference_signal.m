function y = wheel_leg_reference_signal(t)
%WHEEL_LEG_REFERENCE_SIGNAL Simulink-friendly 9x1 joint reference signal.
%
% Output:
%   y = [qd; dqd; ddqd]

[qd, dqd, ddqd] = wheel_leg_reference(t);
y = [qd; dqd; ddqd];
end
