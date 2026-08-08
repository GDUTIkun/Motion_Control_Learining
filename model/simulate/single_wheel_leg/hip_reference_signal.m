function y = hip_reference_signal(t)
%HIP_REFERENCE_SIGNAL Simulink-friendly 6x1 hip reference signal.
%
% Output:
%   y = [x_ref; y_ref; dx_ref; dy_ref; ddx_ref; ddy_ref]

[pRef, dpRef, ddpRef] = hip_reference_trajectory(t);
y = [pRef; dpRef; ddpRef];
end
