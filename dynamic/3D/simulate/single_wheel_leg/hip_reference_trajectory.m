function [pRef, dpRef, ddpRef] = hip_reference_trajectory(t, hip)
%HIP_REFERENCE_TRAJECTORY Smooth hip x/y reference for upper-force testing.
%
% The reference holds the initial hip position, moves by hip.xStep/hip.yStep,
% holds the target, then returns to the initial position with half-cosine
% profiles.

if nargin < 2 || isempty(hip)
    hip = evalin("base", "hip");
end

t = double(t);
holdDuration = getFieldOrDefault(hip, "holdDuration", 0);
returnDuration = getFieldOrDefault(hip, "returnDuration", hip.moveDuration);

tOut = t - hip.settleTime;
tHold = tOut - hip.moveDuration;
tReturn = tHold - holdDuration;

if tOut <= 0
    [alpha, dalpha, ddalpha] = smoothStep(0, 1);
elseif tOut < hip.moveDuration
    [alpha, dalpha, ddalpha] = smoothStep(tOut, hip.moveDuration);
elseif tHold <= holdDuration
    alpha = 1;
    dalpha = 0;
    ddalpha = 0;
elseif tReturn < returnDuration
    [beta, dbeta, ddbeta] = smoothStep(tReturn, returnDuration);
    alpha = 1 - beta;
    dalpha = -dbeta;
    ddalpha = -ddbeta;
else
    alpha = 0;
    dalpha = 0;
    ddalpha = 0;
end

step = [hip.xStep; hip.yStep];
pRef = [hip.xRef; hip.yRef] + step * alpha;
dpRef = [hip.dxRef; hip.dyRef] + step * dalpha;
ddpRef = step * ddalpha;
end

function value = getFieldOrDefault(s, fieldName, defaultValue)
if isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end

function [alpha, dalpha, ddalpha] = smoothStep(t, duration)
if duration <= 0
    alpha = 1;
    dalpha = 0;
    ddalpha = 0;
    return;
end

s = min(max(t / duration, 0), 1);
alpha = 0.5 * (1 - cos(pi * s));
dalpha = 0.5 * pi / duration * sin(pi * s);
ddalpha = 0.5 * (pi / duration)^2 * cos(pi * s);
end
