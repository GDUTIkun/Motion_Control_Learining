function [tau, debug] = spatial_two_leg_qp_core(x)
%SPATIAL_TWO_LEG_QP_CORE 12-DoF inverse-dynamics QP for both wheel legs.
%
% Input layout:
%   [fullNmpcState(18); qL(3); dqL(3); qR(3); dqR(3); ...
%    upperWrench(12); wheelReference(4)]
% fullNmpcState = [t; pB(3); roll; pitch; yaw; vB(3); omegaB(3); ...
%                  xiL; xiR; dxiL; dxiR; wheelHeight].
%
% Generalized coordinates are [pB(3); roll; pitch; yaw; qL(3); qR(3)].
% The QP retains all six floating-base equations and three contact-force
% components per wheel.  Joint axes remain sagittal; underactuation is
% represented by the actuator selection matrix rather than deleting the
% roll/yaw/lateral equations.

persistent qpOptions zWarm rollRecoveryMode

x = double(x(:));
if numel(x) ~= 46
    error("spatial_two_leg_qp_core:InvalidInput", ...
        "Expected the 46D spatial two-leg controller input.");
end

leg = evalin("base", "leg");
base = evalin("base", "base");
ctrl = evalin("base", "ctrl");
traj = evalin("base", "traj");
fullBaseNmpc = evalin("base", "fullBaseNmpc");

t = x(1);
state = x(2:17);
qLeft = x(19:21);
dqLeft = x(22:24);
qRight = x(25:27);
dqRight = x(28:30);
wrenchCommand = x(31:42);
wheelReference = x(43:46);

angles = state(4:6);
[~, ~, eulerRateMap] = rotationData(angles);
omegaPhysical = state([10, 12, 11]);
eulerRates = eulerRateMap \ omegaPhysical;
q = [state([1, 3, 2]); angles; qLeft; qRight];
dq = [state([7, 9, 8]); eulerRates; dqLeft; dqRight];

[M, h, Jc, contactBias, modelData] = spatialDynamics(q, dq, base, leg);
h(7:9) = h(7:9) + ctrl.commonModeJointDamping(:).*dqLeft;
h(10:12) = h(10:12) + ctrl.commonModeJointDamping(:).*dqRight;

upperDerivative = fullBaseNmpc.model.A*state ...
    + fullBaseNmpc.model.B*wrenchCommand ...
    + fullBaseNmpc.model.gravity;
baseQddController = upperDerivative(7:12);
baseQddCommand = baseQddController([1, 3, 2, 4, 5, 6]);

planarState = [state(1); state(3); state(5); ...
    state(7); state(9); state(11)];
planarWrench = [wrenchCommand(1) + wrenchCommand(7); ...
    wrenchCommand(3) + wrenchCommand(9); ...
    wrenchCommand(5) + wrenchCommand(11)];
rH = rotatePitch2D(base.rHBody(:), planarState(3));
planarBaseQdd = [
    planarWrench(1)/fullBaseNmpc.model.m;
    planarWrench(2)/fullBaseNmpc.model.m - base.g;
    (rH(1)*planarWrench(2) - rH(2)*planarWrench(1) ...
        + planarWrench(3))/base.Iyy
];
drdtheta = [-rH(2); rH(1)];
aHCommand = planarBaseQdd(1:2) + planarBaseQdd(3)*drdtheta ...
    - planarState(6)^2*rH;
perLegForce = -base.symmetricLoadShare*planarWrench(1:2);
[qd, dqd, ddqd] = floating_base_leg_reference(t, planarState, ...
    traj, leg, base, aHCommand, perLegForce, true, wheelReference);

commonCtrl = ctrl;
commonCtrl.Kp = getField(ctrl, "commonModeKp", ctrl.Kp);
commonCtrl.Kd = getField(ctrl, "commonModeKd", ctrl.Kd);
qCommon = 0.5*(qLeft + qRight);
dqCommon = 0.5*(dqLeft + dqRight);
qDifferential = 0.5*(qLeft - qRight);
dqDifferential = 0.5*(dqLeft - dqRight);
qddCommonCommand = relativeLegAccelerationCommand(qCommon, dqCommon, ...
    qd, dqd, ddqd, planarState, planarBaseQdd, commonCtrl);
differentialKp = getField(ctrl, "differentialModeKp", commonCtrl.Kp);
differentialKd = getField(ctrl, "differentialModeKd", commonCtrl.Kd);
qddDifferentialCommand = -differentialKp*qDifferential ...
    - differentialKd*dqDifferential;
qddLeftCommand = qddCommonCommand + qddDifferentialCommand;
qddRightCommand = qddCommonCommand - qddDifferentialCommand;

kneeMinLeft = kneeAccelerationLowerBound(qLeft, dqLeft, ctrl);
kneeMinRight = kneeAccelerationLowerBound(qRight, dqRight, ctrl);
qddLeftCommand(2) = max(qddLeftCommand(2), kneeMinLeft);
qddRightCommand(2) = max(qddRightCommand(2), kneeMinRight);

% z = [qdd(12); tauL(3); tauR(3); lambdaL(3); lambdaR(3); slackW(12)].
nq = 12;
ntau = 6;
nlambda = 6;
nwrench = 12;
idxQdd = 1:nq;
idxTau = nq + (1:ntau);
idxLambda = nq + ntau + (1:nlambda);
idxSlack = nq + ntau + nlambda + (1:nwrench);
nz = idxSlack(end);

wBase = getVector(ctrl, "spatialQpWbaseQdd", ...
    1e-3*ones(6, 1), 6);
wCommon = getVector(ctrl, "commonModeQpWqdd", ones(3, 1), 3);
wDifferential = getVector(ctrl, "differentialModeQpWqdd", wCommon, 3);
wTau = repmat(getVector(ctrl, "qpWtau", 1e-5*ones(3, 1), 3), 2, 1);
wLambda = zeros(6, 1);
commonWrenchScale = getVector(ctrl, "spatialQpCommonWrenchScale", ...
    [140; 100; 140; 100; 160; 100], 6);
differentialWrenchScale = getVector(ctrl, ...
    "spatialQpDifferentialWrenchScale", commonWrenchScale, 6);
wrenchPenalty = getField(ctrl, "spatialQpWrenchPenalty", 1e9);
if t <= 0 || isempty(rollRecoveryMode)
    rollRecoveryMode = abs(state(4)) > getField(ctrl, ...
        "spatialQpRollDominantAngle", inf);
end
if rollRecoveryMode
    wBase = getVector(ctrl, "spatialQpRollDominantWbaseQdd", ...
        wBase, 6);
    wrenchPenalty = getField(ctrl, ...
        "spatialQpRollDominantWrenchPenalty", wrenchPenalty);
end
wrenchScale = repmat(commonWrenchScale, 2, 1);

weights = [wBase; zeros(6, 1); wTau; wLambda; zeros(12, 1)];
H = diag(weights) + 1e-9*eye(nz);
f = zeros(nz, 1);
f(1:6) = -wBase.*baseQddCommand;
commonBlock = diag(0.5*(wCommon + wDifferential));
crossBlock = diag(0.5*(wCommon - wDifferential));
H(7:12, 7:12) = [commonBlock, crossBlock; crossBlock, commonBlock] ...
    + 1e-9*eye(6);
f(7:9) = -wCommon.*qddCommonCommand ...
    - wDifferential.*qddDifferentialCommand;
f(10:12) = -wCommon.*qddCommonCommand ...
    + wDifferential.*qddDifferentialCommand;
commonContactWeight = [ctrl.qpWFc(1); 1e-3; ctrl.qpWFc(2)];
differentialContactWeight = [ctrl.differentialModeQpWFc(1); 10; ...
    ctrl.differentialModeQpWFc(2)];
commonContactBlock = diag(0.5*(commonContactWeight ...
    + differentialContactWeight));
crossContactBlock = diag(0.5*(commonContactWeight ...
    - differentialContactWeight));
H(idxLambda, idxLambda) = [commonContactBlock, crossContactBlock; ...
    crossContactBlock, commonContactBlock] + 1e-9*eye(6);
for channel = 1:6
    pair = idxSlack([channel, channel + 6]);
    commonWeight = wrenchPenalty/commonWrenchScale(channel)^2;
    differentialWeight = wrenchPenalty ...
        / differentialWrenchScale(channel)^2;
    H(pair, pair) = H(pair, pair) ...
        + commonWeight*[1, 1; 1, 1] ...
        + 0.25*differentialWeight*[1, -1; -1, 1];
end

S = zeros(nq, ntau);
S(7:9, 1:3) = diag(ctrl.tauSign(:));
S(10:12, 4:6) = diag(ctrl.tauSign(:));
[Dw, Dlambda, wrenchOffset] = interactionWrenchMap( ...
    modelData, base, leg);
H = 0.5*(H + H');
contactRhs = -contactBias ...
    - getField(ctrl, "constraintVelocityGain", 0)*(Jc*dq);
wrenchRhs = wrenchCommand - wrenchOffset;
Aeq = [
    M, -S, -Jc', zeros(nq, nwrench);
    Jc, zeros(6, ntau + nlambda + nwrench);
    Dw, zeros(nwrench, ntau), Dlambda, -eye(nwrench)
];
beq = [-h; contactRhs; wrenchRhs];

[Aineq, bineq] = spatialFrictionConstraints(getField(ctrl, "mu", 0.8), nz, idxLambda);
if isfinite(kneeMinLeft)
    row = zeros(1, nz);
    row(8) = -1;
    Aineq = [Aineq; row];
    bineq = [bineq; -kneeMinLeft];
end
if isfinite(kneeMinRight)
    row = zeros(1, nz);
    row(11) = -1;
    Aineq = [Aineq; row];
    bineq = [bineq; -kneeMinRight];
end

tauMax = repmat(ctrl.tauMax(:), 2, 1);
lb = -inf(nz, 1);
ub = inf(nz, 1);
lb(idxTau) = -tauMax;
ub(idxTau) = tauMax;
lb(idxLambda([3, 6])) = 0;

robotMass = base.body.mass + 2*(leg.m1 + leg.m2 + leg.mw);
z0 = zeros(nz, 1);
z0(idxQdd) = [baseQddCommand; qddLeftCommand; qddRightCommand];
z0(idxLambda([3, 6])) = robotMass*base.g/2;
if t <= 0 || isempty(zWarm) || numel(zWarm) ~= nz || any(~isfinite(zWarm))
    zWarm = z0;
elseif getField(ctrl, "qpWarmStart", true)
    z0 = zWarm;
end

exitflag = -999;
% With inactive friction/torque bounds, the equality-constrained KKT point
% is already the exact QP optimum and is more reliable than an iterative
% solve of this strongly weighted hierarchical objective.
[z, kktFlag] = solveEqualityQp(H, f, Aeq, beq);
candidateUsable = isUsableQpCandidate( ...
    z, Aeq, beq, Aineq, bineq, lb, ub, 1e-4);
if candidateUsable
    exitflag = kktFlag;
end
try
    if ~candidateUsable && isempty(qpOptions)
        qpOptions = optimoptions("quadprog", "Display", "off", ...
            "Algorithm", "interior-point-convex");
    end
    if ~candidateUsable
        [z, ~, exitflag] = quadprog(H, f, Aineq, bineq, Aeq, beq, ...
            lb, ub, z0, qpOptions);
    end
catch
    z = [];
end
candidateUsable = isUsableQpCandidate( ...
    z, Aeq, beq, Aineq, bineq, lb, ub, 1e-4);
if ~candidateUsable
    [z, fallbackFlag] = solveEqualityQp(H, f, Aeq, beq);
    if isempty(z)
        z = z0;
    end
    z(idxTau) = min(max(z(idxTau), -tauMax), tauMax);
    z(idxLambda([3, 6])) = max(z(idxLambda([3, 6])), 0);
    exitflag = min(-1, fallbackFlag);
else
    zWarm = z;
    exitflag = max(1, exitflag);
end

qddSolution = z(idxQdd);
tau = z(idxTau);
lambda = z(idxLambda);
wrenchSlack = z(idxSlack);
wrenchFeasible = Dw*qddSolution + Dlambda*lambda + wrenchOffset;
eqResidual = Aeq*z - beq;
ineqResidual = Aineq*z - bineq;
physicalToController = [1, 0, 0; 0, 0, 1; 0, 1, 0];
contactForceLeft = physicalToController ...
    * modelData.contactBasis(:, :, 1)*lambda(1:3);
contactForceRight = physicalToController ...
    * modelData.contactBasis(:, :, 2)*lambda(4:6);
muPyramid = getField(ctrl, "mu", 0.8)/sqrt(2);
frictionMargin = [
    muPyramid*lambda(3) - abs(lambda(1));
    muPyramid*lambda(3) - abs(lambda(2));
    muPyramid*lambda(6) - abs(lambda(4));
    muPyramid*lambda(6) - abs(lambda(5))
];

wheelLeft = modelData.wheelBody(1);
wheelRight = modelData.wheelBody(2);
tangent = modelData.contactBasis(:, 1, 1);
Jxi = 0.5*tangent'*(modelData.Jv(:, :, wheelLeft) ...
    - modelData.Jv(:, :, wheelRight));
xiDifferential = 0.5*tangent'*(modelData.position(:, wheelLeft) ...
    - modelData.position(:, wheelRight));
dxiDifferential = Jxi*dq;
xiBias = 0.5*tangent'*(modelData.biasV(:, wheelLeft) ...
    - modelData.biasV(:, wheelRight));
xiDifferentialCommand = -getField(ctrl, ...
    "differentialWheelPositionKp", 0)*xiDifferential ...
    - getField(ctrl, "differentialWheelPositionKd", 0)*dxiDifferential;

debug = struct();
debug.qdd = qddSolution;
debug.qddBase = qddSolution([1, 3, 2, 4, 5, 6]);
debug.qddDifferential = 0.5*(qddSolution(7:9) - qddSolution(10:12));
debug.qddDifferentialCommand = qddDifferentialCommand;
debug.tauDifferential = 0.5*(tau(1:3) - tau(4:6));
debug.contactForceDifferential = 0.5*(contactForceLeft - contactForceRight);
debug.FcLeft = contactForceLeft;
debug.FcRight = contactForceRight;
debug.lambdaLeft = lambda(1:3);
debug.lambdaRight = lambda(4:6);
debug.wrenchCommand = wrenchCommand;
debug.wrenchSlack = wrenchSlack;
debug.wrenchFeasible = wrenchFeasible;
debug.wrenchSlackNorm = norm(wrenchSlack./wrenchScale);
debug.wrenchResidual = wrenchFeasible - wrenchCommand - wrenchSlack;
debug.exitflag = exitflag;
debug.dynamicsResidual = M*qddSolution + h - S*tau - Jc'*lambda;
debug.contactResidual = Jc*qddSolution - contactRhs;
debug.qpFeasible = exitflag > 0 && norm(eqResidual, inf) < 1e-4 ...
    && (isempty(ineqResidual) || max(ineqResidual) < 1e-6);
debug.frictionMargin = frictionMargin;
debug.torqueMargin = tauMax - abs(tau);
debug.xiDifferential = xiDifferential;
debug.dxiDifferential = dxiDifferential;
debug.xiDifferentialAcceleration = Jxi*qddSolution + xiBias;
debug.xiDifferentialCommand = xiDifferentialCommand;
debug.massMatrix = M;
debug.spatialQp = true;
end

function [M, h, Jc, contactBias, data] = spatialDynamics(q, dq, base, leg)
[M, data] = spatialMassMatrix(q, dq, base, leg);
n = numel(q);
dM = zeros(n, n, n);
step = 1e-6;
for k = 4:n
    delta = zeros(n, 1);
    delta(k) = step;
    Mplus = spatialMassMatrix(q + delta, zeros(n, 1), base, leg);
    Mminus = spatialMassMatrix(q - delta, zeros(n, 1), base, leg);
    dM(:, :, k) = (Mplus - Mminus)/(2*step);
end
Mdot = zeros(n);
kineticGradient = zeros(n, 1);
for k = 1:n
    Mdot = Mdot + dM(:, :, k)*dq(k);
    kineticGradient(k) = dq'*dM(:, :, k)*dq;
end
coriolis = Mdot*dq - 0.5*kineticGradient;
gravity = zeros(n, 1);
for body = 1:numel(data.mass)
    gravity = gravity + data.Jv(:, :, body)' ...
        *(data.mass(body)*[0; base.g; 0]);
end
h = coriolis + gravity;

Jc = contactJacobian(data, leg);
if norm(dq, inf) == 0
    contactBias = zeros(6, 1);
    data.biasV = zeros(3, numel(data.mass));
    data.biasW = zeros(3, numel(data.mass));
else
    dt = 1e-6/max(1, norm(dq, inf));
    [~, plus] = spatialMassMatrix(q + dt*dq, dq, base, leg);
    [~, minus] = spatialMassMatrix(q - dt*dq, dq, base, leg);
    Jplus = contactJacobian(plus, leg);
    Jminus = contactJacobian(minus, leg);
    contactBias = ((Jplus - Jminus)/(2*dt))*dq;
    bodyCount = numel(data.mass);
    data.biasV = zeros(3, bodyCount);
    data.biasW = zeros(3, bodyCount);
    for body = 1:bodyCount
        data.biasV(:, body) = ((plus.Jv(:, :, body) ...
            - minus.Jv(:, :, body))/(2*dt))*dq;
        data.biasW(:, body) = ((plus.Jw(:, :, body) ...
            - minus.Jw(:, :, body))/(2*dt))*dq;
    end
end
end

function [M, data] = spatialMassMatrix(q, dq, base, leg)
n = 12;
bodyCount = 7;
data.mass = zeros(bodyCount, 1);
data.position = zeros(3, bodyCount);
data.Jv = zeros(3, n, bodyCount);
data.Jw = zeros(3, n, bodyCount);
data.inertia = zeros(3, 3, bodyCount);
data.omega = zeros(3, bodyCount);
data.wheelBody = [4, 7];

p = q(1:3);
angles = q(4:6);
[R, dR, E] = rotationData(angles);
baseInertia = diag(base.body.mass/12 * [
    base.body.widthY^2 + base.body.heightZ^2;
    base.body.lengthX^2 + base.body.widthY^2;
    base.body.lengthX^2 + base.body.heightZ^2]);
data.mass(1) = base.body.mass;
data.position(:, 1) = p;
data.Jv(:, 1:3, 1) = eye(3);
data.Jw(:, 4:6, 1) = E;
data.inertia(:, :, 1) = R*baseInertia*R';
data.omega(:, 1) = data.Jw(:, :, 1)*dq;

link1Inertia = linkInertia(leg.m1, leg.L1, leg);
link2Inertia = linkInertia(leg.m2, leg.L2, leg);
wheelTransverse = leg.mw*(3*leg.r^2 + leg.width^2)/12;
wheelInertia = diag([wheelTransverse; wheelTransverse; leg.Iw]);
for side = 1:2
    if side == 1
        first = 7;
        hipBody = base.body.hipPositionBodyLeft3D(:);
        bodyFirst = 2;
    else
        first = 10;
        hipBody = base.body.hipPositionBodyRight3D(:);
        bodyFirst = 5;
    end
    qh = q(first);
    qhk = q(first) + q(first + 1);
    qWheel = qhk + q(first + 2);
    localPoints = [
        leg.c1*sin(qh), leg.L1*sin(qh) + leg.c2*sin(qhk), ...
            leg.L1*sin(qh) + leg.L2*sin(qhk);
        -leg.c1*cos(qh), -leg.L1*cos(qh) - leg.c2*cos(qhk), ...
            -leg.L1*cos(qh) - leg.L2*cos(qhk);
        0, 0, 0
    ];
    localDerivatives = zeros(3, 3, 3);
    localDerivatives(:, 1, 1) = [leg.c1*cos(qh); leg.c1*sin(qh); 0];
    localDerivatives(:, 1, 2) = [leg.L1*cos(qh) + leg.c2*cos(qhk); ...
        leg.L1*sin(qh) + leg.c2*sin(qhk); 0];
    localDerivatives(:, 2, 2) = [leg.c2*cos(qhk); leg.c2*sin(qhk); 0];
    localDerivatives(:, 1, 3) = [leg.L1*cos(qh) + leg.L2*cos(qhk); ...
        leg.L1*sin(qh) + leg.L2*sin(qhk); 0];
    localDerivatives(:, 2, 3) = [leg.L2*cos(qhk); leg.L2*sin(qhk); 0];
    bodyAngles = [qh, qhk, qWheel];
    bodyMasses = [leg.m1, leg.m2, leg.mw];
    bodyInertias = cat(3, link1Inertia, link2Inertia, wheelInertia);
    for localBody = 1:3
        body = bodyFirst + localBody - 1;
        rBody = hipBody + localPoints(:, localBody);
        data.mass(body) = bodyMasses(localBody);
        data.position(:, body) = p + R*rBody;
        data.Jv(:, 1:3, body) = eye(3);
        for k = 1:3
            data.Jv(:, 3 + k, body) = dR(:, :, k)*rBody;
        end
        data.Jv(:, first, body) = R*localDerivatives(:, 1, localBody);
        if localBody >= 2
            data.Jv(:, first + 1, body) = R*localDerivatives(:, 2, localBody);
        end
        data.Jw(:, 4:6, body) = E;
        axisWorld = R*[0; 0; 1];
        data.Jw(:, first, body) = axisWorld;
        if localBody >= 2
            data.Jw(:, first + 1, body) = axisWorld;
        end
        if localBody == 3
            data.Jw(:, first + 2, body) = axisWorld;
        end
        bodyRotation = R*rotationZ(bodyAngles(localBody));
        data.inertia(:, :, body) = bodyRotation ...
            * bodyInertias(:, :, localBody)*bodyRotation';
        data.omega(:, body) = data.Jw(:, :, body)*dq;
    end
end

M = zeros(n);
for body = 1:bodyCount
    M = M + data.mass(body)*(data.Jv(:, :, body)'*data.Jv(:, :, body)) ...
        + data.Jw(:, :, body)'*data.inertia(:, :, body)*data.Jw(:, :, body);
end
M = 0.5*(M + M');

forward = R*[1; 0; 0];
tangent = [forward(1); 0; forward(3)];
if norm(tangent) < 1e-9
    tangent = [1; 0; 0];
else
    tangent = tangent/norm(tangent);
end
lateral = cross(tangent, [0; 1; 0]);
normal = [0; 1; 0];
data.contactBasis = zeros(3, 3, 2);
data.axle = zeros(3, 2);
for side = 1:2
    data.contactBasis(:, :, side) = [tangent, lateral, normal];
    data.axle(:, side) = R*[0; 0; 1];
end
end

function Jc = contactJacobian(data, leg)
Jc = zeros(6, 12);
for side = 1:2
    body = data.wheelBody(side);
    basis = data.contactBasis(:, :, side);
    axle = data.axle(:, side);
    rows = 3*side - 2:3*side;
    Jc(rows(1), :) = basis(:, 1)'*data.Jv(:, :, body) ...
        + leg.r*axle'*data.Jw(:, :, body);
    Jc(rows(2), :) = basis(:, 2)'*data.Jv(:, :, body);
    Jc(rows(3), :) = basis(:, 3)'*data.Jv(:, :, body);
end
end

function [Dw, Dlambda, offset] = interactionWrenchMap(data, base, leg)
Dw = zeros(12, 12);
Dlambda = zeros(12, 6);
offset = zeros(12, 1);
for side = 1:2
    if side == 1
        wheelBody = 4;
    else
        wheelBody = 7;
    end
    rows = 6*side - 5:6*side;
    mass = data.mass(wheelBody);
    Jv = data.Jv(:, :, wheelBody);
    Jw = data.Jw(:, :, wheelBody);
    inertia = data.inertia(:, :, wheelBody);
    gravityForce = [0; -mass*base.g; 0];
    % The upper model lumps the torso and both links into its floating body.
    % Its per-side interface is therefore the wheel-to-rest-of-robot wrench
    % at the wheel centre, not the complete leg-on-torso hip wrench.
    physicalDw = [-mass*Jv; -inertia*Jw];
    physicalOffset = [gravityForce - mass*data.biasV(:, wheelBody); ...
        -inertia*data.biasW(:, wheelBody) ...
        - cross(data.omega(:, wheelBody), inertia*data.omega(:, wheelBody))];
    columns = 3*side - 2:3*side;
    basis = data.contactBasis(:, :, side);
    physicalDlambda = [basis; zeros(3, 3)];
    physicalDlambda(4:6, 1) = leg.r*data.axle(:, side);
    physicalToController = [1, 0, 0; 0, 0, 1; 0, 1, 0];
    wrenchTransform = blkdiag(physicalToController, physicalToController);
    Dw(rows, :) = wrenchTransform*physicalDw;
    Dlambda(rows, columns) = wrenchTransform*physicalDlambda;
    offset(rows) = wrenchTransform*physicalOffset;
end
end

function [A, b] = spatialFrictionConstraints(mu, nz, idxLambda)
muPyramid = mu/sqrt(2);
A = zeros(8, nz);
b = zeros(8, 1);
for side = 1:2
    columns = idxLambda(3*side - 2:3*side);
    row = 4*side - 3;
    A(row, columns) = [1, 0, -muPyramid];
    A(row + 1, columns) = [-1, 0, -muPyramid];
    A(row + 2, columns) = [0, 1, -muPyramid];
    A(row + 3, columns) = [0, -1, -muPyramid];
end
end

function qddCommand = relativeLegAccelerationCommand(q, dq, qd, dqd, ...
        ddqd, baseState, baseQddCommand, ctrl)
qdRelative = [qd(1) - baseState(3); qd(2:3)];
dqdRelative = [dqd(1) - baseState(6); dqd(2:3)];
ddqdRelative = [ddqd(1) - baseQddCommand(3); ddqd(2:3)];
qddCommand = ddqdRelative + ctrl.Kd*(dqdRelative - dq) ...
    + ctrl.Kp*(qdRelative - q);
end

function lowerBound = kneeAccelerationLowerBound(q, dq, ctrl)
if ~getField(ctrl, "kneeGuardEnabled", false) || q(2) >= ctrl.kneeGuardMin
    lowerBound = -inf;
    return;
end
wn = 2*pi*ctrl.kneeGuardFrequencyHz;
lowerBound = -2*ctrl.kneeGuardDamping*wn*dq(2) ...
    - wn^2*(q(2) - ctrl.kneeGuardMin);
end

function [z, exitflag] = solveEqualityQp(H, f, Aeq, beq)
regularization = 1e-9;
rowScale = max(vecnorm(Aeq, 2, 2), 1e-9);
scaledAeq = Aeq./rowScale;
scaledBeq = beq./rowScale;
KKT = [H + regularization*eye(size(H)), scaledAeq'; ...
    scaledAeq, zeros(size(scaledAeq, 1))];
rhs = [-f; scaledBeq];
if rcond(KKT) < 1e-14
    solution = pinv(KKT)*rhs;
else
    solution = KKT\rhs;
end
z = solution(1:size(H, 1));
exitflag = double(all(isfinite(z)));
if exitflag == 0
    z = [];
end
end

function usable = isUsableQpCandidate( ...
        z, Aeq, beq, Aineq, bineq, lb, ub, tolerance)
usable = ~isempty(z) && all(isfinite(z));
if ~usable
    return;
end
usable = norm(Aeq*z - beq, inf) <= tolerance ...
    && (isempty(Aineq) || max(Aineq*z - bineq) <= tolerance) ...
    && max(lb - z) <= tolerance ...
    && max(z - ub) <= tolerance;
end

function inertia = linkInertia(mass, lengthValue, leg)
inertia = diag(mass/12 * [
    lengthValue^2 + leg.depth^2;
    leg.width^2 + leg.depth^2;
    lengthValue^2 + leg.width^2]);
end

function [R, dR, E] = rotationData(angles)
roll = angles(1);
pitch = angles(2);
yaw = angles(3);
Rx = [1, 0, 0; 0, cos(roll), -sin(roll); 0, sin(roll), cos(roll)];
Ry = [cos(yaw), 0, sin(yaw); 0, 1, 0; -sin(yaw), 0, cos(yaw)];
Rz = rotationZ(pitch);
dRx = [0, 0, 0; 0, -sin(roll), -cos(roll); 0, cos(roll), -sin(roll)];
dRy = [-sin(yaw), 0, cos(yaw); 0, 0, 0; ...
    -cos(yaw), 0, -sin(yaw)];
dRz = [-sin(pitch), -cos(pitch), 0; ...
    cos(pitch), -sin(pitch), 0; 0, 0, 0];
R = Rz*Ry*Rx;
dR = cat(3, Rz*Ry*dRx, dRz*Ry*Rx, Rz*dRy*Rx);
E = [Rz*Ry*[1; 0; 0], [0; 0; 1], Rz*[0; 1; 0]];
end

function R = rotationZ(angle)
R = [cos(angle), -sin(angle), 0; ...
    sin(angle), cos(angle), 0; 0, 0, 1];
end

function rWorld = rotatePitch2D(rBody, theta)
rWorld = [cos(theta)*rBody(1) - sin(theta)*rBody(2); ...
    sin(theta)*rBody(1) + cos(theta)*rBody(2)];
end

function value = getField(s, name, fallback)
if isfield(s, name)
    value = s.(name);
else
    value = fallback;
end
end

function value = getVector(s, name, fallback, count)
value = getField(s, name, fallback);
value = value(:);
if numel(value) ~= count
    error("spatial_two_leg_qp_core:InvalidWeight", ...
        "%s must contain %d elements.", name, count);
end
end
