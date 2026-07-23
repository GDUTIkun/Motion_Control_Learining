function tau = controller(x)
%CONTROLLER Dynamics feedforward plus PD for one wheel-leg.
%
% Interpreted MATLAB Fcn input:
%   x = [t; qh; qk; qw; dqh; dqk; dqw; FHx_ext; FHz_ext]
%
% Output:
%   tau = [tau_h; tau_k; tau_w]

if numel(x) ~= 9 && numel(x) ~= 11
    error("controller:InvalidInput", ...
        "Expected x = [t; qh; qk; qw; dqh; dqk; dqw; FHx_ext; FHz_ext].");
end

leg = evalin("base", "leg");
ctrl = evalin("base", "ctrl");
traj = evalin("base", "traj");

x = double(x(:));
t = x(1);
q = x(2:4);
dq = x(5:7);
if numel(x) == 11
    % Legacy wiring was [t;q;dq;Fcx;Fcz;FH_ext]. Fc is no longer used:
    % the contact reaction is eliminated as the constraint multiplier.
    FH_ext = x(10:11);
else
    FH_ext = x(8:9);
end

[qd, dqd, ddqd] = wheel_leg_reference(t, traj, leg);
tracking_accel = ddqd + ctrl.Kd * (dqd - dq) + ctrl.Kp * (qd - q);

[M, C, G] = wheel_leg_dynamics(q, dq, leg);
kin = wheel_leg_kinematics(q, dq, [], leg);

% Fc is not an input. In the Simscape plant, wheel-ground contact generates
% the unknown constraint reaction. The controller therefore compensates the
% modeled inertia, velocity, gravity, and hip external-load terms only.
tau = M * tracking_accel + C + G - kin.JH' * FH_ext;

tau = min(max(tau, -ctrl.tauMax), ctrl.tauMax);
tau = tau(:);
end
