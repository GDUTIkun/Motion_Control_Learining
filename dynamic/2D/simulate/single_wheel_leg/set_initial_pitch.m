function [base, leg, hip, baseLqr] = set_initial_pitch(theta0, dtheta0)
%SET_INITIAL_PITCH Convenience wrapper for initial pitch disturbances.

if nargin < 1 || isempty(theta0)
    theta0 = 0;
end
if nargin < 2 || isempty(dtheta0)
    dtheta0 = 0;
end

x0 = [0; 0; double(theta0); 0; 0; double(dtheta0)];
[base, leg, hip, baseLqr] = set_initial_base_state(x0);
end
