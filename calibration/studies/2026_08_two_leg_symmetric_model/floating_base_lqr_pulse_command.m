function y = floating_base_lqr_pulse_command(x)
%FLOATING_BASE_LQR_PULSE_COMMAND Add a test pulse to the closed-loop LQR.

y = floating_base_lqr_command(x);
config = evalin("base", "wrenchPulse");
t = double(x(1));
if t >= config.startTime && t < config.startTime + config.duration
    % The production command uses body-on-leg force for its first two rows.
    y(1) = y(1) - config.fxBody;
end
end
