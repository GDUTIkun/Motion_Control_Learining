function tau = controller_constraint(x)
%CONTROLLER_CONSTRAINT Inverse dynamics with internal contact constraints.
%
% Interpreted MATLAB Fcn input:
%   x = [t; qh; qk; qw; dqh; dqk; dqw; FHx_ext; FHy_ext]
%
% Output:
%   tau = [tau_h; tau_k; tau_w]

[tau, ~, ~] = controller_constraint_core(x);
end
