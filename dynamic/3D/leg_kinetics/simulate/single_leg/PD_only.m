function tau = PD_only(x)
%PD_ONLY Pure PD tracking controller.
%
% Interpreted MATLAB Fcn input:
%   x = [t; qh; qk; dqh; dqk]

tau = two_link_leg_controller("pd", x);
end
