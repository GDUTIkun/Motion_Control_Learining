function [xRef, aRef] = floating_base_reference(t, baseLqr)
%FLOATING_BASE_REFERENCE Configurable round-trip planar base reference.
%
% xRef = [x; z; theta; dx; dz; dtheta]
% aRef = [ddx; ddz; ddtheta]

if nargin < 2 || isempty(baseLqr)
    baseLqr = evalin("base", "baseLqr");
end

xRef = baseLqr.xRef(:);
aRef = zeros(3, 1);
trajectory = getFieldOrDefault(baseLqr, "trajectory", struct());
if ~getFieldOrDefault(trajectory, "enabled", false) || ~isfinite(t)
    return;
end

mode = lower(string(getFieldOrDefault(trajectory, "mode", "position")));
if mode == "velocity_round_trip"
    [xOffset, dxRef, ddxRef] = velocityRoundTrip(t, trajectory);
    xRef(1) = xRef(1) + xOffset;
    xRef(4) = xRef(4) + dxRef;
    aRef(1) = ddxRef;
    return;
end

settleTime = getFieldOrDefault(trajectory, "settleTime", 0);
moveDuration = getFieldOrDefault(trajectory, "moveDuration", 0);
holdDuration = getFieldOrDefault(trajectory, "holdDuration", 0);
returnDuration = getFieldOrDefault(trajectory, "returnDuration", moveDuration);
xStep = getFieldOrDefault(trajectory, "xStep", 0);
zStep = getFieldOrDefault(trajectory, "zStep", 0);

tMove = t - settleTime;
tReturn = tMove - moveDuration - holdDuration;
if tMove <= 0
    [alpha, dalpha, ddalpha] = smoothStep(0, 1);
elseif tMove < moveDuration
    [alpha, dalpha, ddalpha] = smoothStep(tMove, moveDuration);
elseif tReturn <= 0
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

xRef(1) = xRef(1) + xStep * alpha;
xRef(2) = xRef(2) + zStep * alpha;
xRef(4) = xRef(4) + xStep * dalpha;
xRef(5) = xRef(5) + zStep * dalpha;
aRef(1) = xStep * ddalpha;
aRef(2) = zStep * ddalpha;
end

function [x, dx, ddx] = velocityRoundTrip(t, trajectory)
settleTime = getFieldOrDefault(trajectory, "settleTime", 0);
speed = getFieldOrDefault(trajectory, "cruiseVelocity", 0);
accelDuration = getFieldOrDefault(trajectory, "accelDuration", 0);
cruiseDuration = getFieldOrDefault(trajectory, "cruiseDuration", 0);
decelDuration = getFieldOrDefault(trajectory, "decelDuration", accelDuration);
turnHoldDuration = getFieldOrDefault(trajectory, "turnHoldDuration", 0);

segmentDuration = accelDuration + cruiseDuration + decelDuration;
forwardDistance = speed * (cruiseDuration ...
    + 0.5 * (accelDuration + decelDuration));
tForward = t - settleTime;

if tForward <= segmentDuration + turnHoldDuration
    [x, dx, ddx] = velocitySegment(tForward, speed, ...
        accelDuration, cruiseDuration, decelDuration);
else
    [xBack, dx, ddx] = velocitySegment( ...
        tForward - segmentDuration - turnHoldDuration, -speed, ...
        accelDuration, cruiseDuration, decelDuration);
    x = forwardDistance + xBack;
end
end

function [x, dx, ddx] = velocitySegment(t, speed, accelDuration, ...
        cruiseDuration, decelDuration)
if t <= 0
    x = 0;
    dx = 0;
    ddx = 0;
elseif t < accelDuration
    [alpha, dalpha] = smoothStep(t, accelDuration);
    s = t / accelDuration;
    x = speed * accelDuration * smoothStepIntegral(s);
    dx = speed * alpha;
    ddx = speed * dalpha;
elseif t < accelDuration + cruiseDuration
    x = speed * (0.5 * accelDuration + t - accelDuration);
    dx = speed;
    ddx = 0;
elseif t < accelDuration + cruiseDuration + decelDuration
    tDecel = t - accelDuration - cruiseDuration;
    [beta, dbeta] = smoothStep(tDecel, decelDuration);
    s = tDecel / decelDuration;
    x = speed * (0.5 * accelDuration + cruiseDuration ...
        + tDecel - decelDuration * smoothStepIntegral(s));
    dx = speed * (1 - beta);
    ddx = -speed * dbeta;
else
    x = speed * (cruiseDuration ...
        + 0.5 * (accelDuration + decelDuration));
    dx = 0;
    ddx = 0;
end
end

function value = smoothStepIntegral(s)
value = 2.5*s^4 - 3*s^5 + s^6;
end

function [alpha, dalpha, ddalpha] = smoothStep(t, duration)
if duration <= 0
    alpha = 1;
    dalpha = 0;
    ddalpha = 0;
    return;
end

s = min(max(t / duration, 0), 1);
alpha = 10*s^3 - 15*s^4 + 6*s^5;
dalpha = (30*s^2 - 60*s^3 + 30*s^4) / duration;
ddalpha = (60*s - 180*s^2 + 120*s^3) / duration^2;
end

function value = getFieldOrDefault(s, name, defaultValue)
if isstruct(s) && isfield(s, name)
    value = s.(name);
else
    value = defaultValue;
end
end
