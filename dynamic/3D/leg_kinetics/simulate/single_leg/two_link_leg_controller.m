function tau = two_link_leg_controller(mode, x)
%TWO_LINK_LEG_CONTROLLER Shared implementation for Interpreted MATLAB Fcn.
%
% Input:
%   x = [t; qh; qk; dqh; dqk]
%
% Output:
%   tau = [tau_h; tau_k]

if numel(x) ~= 5
    error("two_link_leg_controller:InvalidInput", ...
        "Expected x = [t; qh; qk; dqh; dqk].");
end

leg = evalin("base", "leg");
ctrl = evalin("base", "ctrl");
traj = evalin("base", "traj");

x = double(x(:));
t = x(1);
q = x(2:3);
dq = x(4:5);

[qd, dqd, ddqd] = two_link_leg_reference(t, traj);
tau_pd = ctrl.Kp * (qd - q) + ctrl.Kd * (dqd - dq);

switch lower(string(mode))
    case "pd"
        tau = tau_pd;
    case {"dynamic", "computed_torque", "feedforward"}
        [M, C, G] = two_link_leg_dynamics(q, dq, leg);
        tau = M * ddqd + C + G + tau_pd;
    otherwise
        error("two_link_leg_controller:InvalidMode", ...
            "Unknown controller mode '%s'.", mode);
end

tau = min(max(tau, -ctrl.tauMax), ctrl.tauMax);
tau = tau(:);
end
