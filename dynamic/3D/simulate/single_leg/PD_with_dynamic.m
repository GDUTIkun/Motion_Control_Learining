function tau = PD_with_dynamic(x)
%PD_WITH_DYNAMIC Dynamics feedforward plus PD tracking controller.
%
% Interpreted MATLAB Fcn input:
%   x = [t; qh; qk; dqh; dqk]

tau = two_link_leg_controller("dynamic", x);
end
