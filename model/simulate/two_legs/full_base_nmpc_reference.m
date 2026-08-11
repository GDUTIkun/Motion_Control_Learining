function reference = full_base_nmpc_reference(x, baseLqr, config, wheelLqr)
%FULL_BASE_NMPC_REFERENCE Build 16-state/12-input horizon references.

if nargin < 2 || isempty(baseLqr)
    baseLqr = evalin("base", "baseLqr");
end
if nargin < 3 || isempty(config)
    config = evalin("base", "fullBaseNmpc");
end
if nargin < 4 || isempty(wheelLqr)
    wheelLqr = evalin("base", "wheelLqr");
end
x = double(x(:));
if numel(x) ~= 17
    error("full_base_nmpc_reference:InvalidInput", ...
        "Expected [t; planner(4); previousWrench(12)].");
end
t = x(1);
planner = x(2:5);
previousWrench = x(6:17);
N = config.N;
stageSize = 40;
pathReference = zeros(stageSize, max(N - 1, 0));
xiRef = planner(1);
dxiRef = planner(2);
xiRaw = planner(4);

for k = 0:N
    [baseReference, aRef] = floating_base_reference( ...
        t + k*config.Ts, baseLqr);
    turningReference = turning_motion_reference(t + k*config.Ts, ...
        baseReference(4), baseLqr.trajectory, config.model.halfTrack);
    stateReference = [
        baseReference(1); 0; baseReference(2);
        0; baseReference(3); turningReference(1);
        baseReference(4); 0; baseReference(5);
        0; baseReference(6); turningReference(2);
        xiRef; xiRef; dxiRef; dxiRef
    ];
    planarWrench = feedforwardWrench(aRef, xiRef, config.model);
    yawMoment = config.model.inertia(3)*turningReference(3);
    yawForceDifference = yawMoment/(2*config.model.halfTrack);
    leftWrench = [planarWrench(1)/2 - yawForceDifference; 0; ...
        planarWrench(2)/2; 0; planarWrench(3)/2; 0];
    rightWrench = [planarWrench(1)/2 + yawForceDifference; 0; ...
        planarWrench(2)/2; 0; planarWrench(3)/2; 0];
    uRef = [leftWrench; rightWrench];
    uRef = min(max(uRef, config.uMin(:)), config.uMax(:));
    stageReference = [stateReference; uRef; previousWrench];
    if k == 0
        initialReference = stageReference;
    elseif k < N
        pathReference(:, k) = stageReference;
    else
        terminalReference = stateReference;
    end
    if k < N
        [xiRef, dxiRef] = wheel_position_governor_step( ...
            xiRef, dxiRef, xiRaw, config.Ts, wheelLqr);
    end
end
reference = [initialReference; pathReference(:); terminalReference];
end

function u = feedforwardWrench(aRef, xiRef, model)
Fx = model.m*aRef(1);
Fz = model.m*(model.g + aRef(2));
My = model.inertia(2)*aRef(3) ...
    - (xiRef - model.xiEq)*Fz + model.rWzEq*Fx;
u = [Fx; Fz; My];
end
