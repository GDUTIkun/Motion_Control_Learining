function y = hip_force_reference(x, hip)
%HIP_FORCE_REFERENCE Legacy wrapper for hip_position_pd.
%
% Prefer hip_position_pd([t;xH;yH;dxH;dyH], hip). If this function is called
% with only time, it evaluates the command at the reference state.

if nargin < 2 || isempty(hip)
    hip = evalin("base", "hip");
end

x = double(x(:));
if numel(x) == 1
    [pRef, dpRef] = hip_reference_trajectory(x(1), hip);
    x = [x(1); pRef; dpRef];
end

y = hip_position_pd(x, hip);
end
