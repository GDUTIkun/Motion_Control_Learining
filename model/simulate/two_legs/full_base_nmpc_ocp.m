function ocp = full_base_nmpc_ocp(base, leg, fullBaseNmpc)
%FULL_BASE_NMPC_OCP Build the 16-state, 12-input 8-DoF acados OCP.

arguments
    base (1, 1) struct
    leg (1, 1) struct
    fullBaseNmpc (1, 1) struct
end

import casadi.*

x = SX.sym('x', 16, 1);
u = SX.sym('u', 12, 1);
xdot = SX.sym('xdot', 16, 1);
m = fullBaseNmpc.model.m;
I = fullBaseNmpc.model.inertia;
d = fullBaseNmpc.model.halfTrack;
h = fullBaseNmpc.model.rWzEq;
xiEq = fullBaseNmpc.model.xiEq;
wheelDenominator = fullBaseNmpc.model.rollingDenominator;

FL = u(1:3);
TL = u(4:6);
FR = u(7:9);
TR = u(10:12);
force = FL + FR;
rollMoment = d*(FL(3) - FR(3)) - h*(FL(2) + FR(2)) ...
    + TL(1) + TR(1);
pitchMoment = (x(13) - xiEq)*FL(3) ...
    + (x(14) - xiEq)*FR(3) - h*(FL(1) + FR(1)) ...
    + TL(2) + TR(2);
yawMoment = -d*FL(1) + d*FR(1) ...
    + x(13)*FL(2) + x(14)*FR(2) + TL(3) + TR(3);
xiAccelerationLeft = -force(1)/m ...
    - (leg.r*FL(1) + TL(2))/wheelDenominator;
xiAccelerationRight = -force(1)/m ...
    - (leg.r*FR(1) + TR(2))/wheelDenominator;

fExpl = [
    x(7:12);
    force(1)/m;
    force(2)/m;
    force(3)/m - base.g;
    rollMoment/I(1);
    pitchMoment/I(2);
    yawMoment/I(3);
    x(15:16);
    xiAccelerationLeft;
    xiAccelerationRight
];

model = AcadosModel();
model.name = 'full_base_two_wheel_leg_body';
model.x = x;
model.u = u;
model.xdot = xdot;
model.f_expl_expr = fExpl;
model.f_impl_expr = xdot - fExpl;
mu = fullBaseNmpc.driveCoefficient;
model.con_h_expr = [u(1) - mu*u(3); -u(1) - mu*u(3); ...
    u(7) - mu*u(9); -u(7) - mu*u(9)];

stageCost = vertcat(x, u, u);
W = blkdiag(fullBaseNmpc.Q, fullBaseNmpc.R1, fullBaseNmpc.R2);
ocp = AcadosOcp();
ocp.name = char(fullBaseNmpc.solverName);
ocp.model = model;
ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.model.cost_y_expr_0 = stageCost;
ocp.cost.W_0 = W;
ocp.cost.yref_0 = zeros(40, 1);
ocp.cost.cost_type = 'NONLINEAR_LS';
ocp.model.cost_y_expr = stageCost;
ocp.cost.W = W;
ocp.cost.yref = zeros(40, 1);
ocp.cost.cost_type_e = 'NONLINEAR_LS';
ocp.model.cost_y_expr_e = x;
ocp.cost.W_e = fullBaseNmpc.W_e;
ocp.cost.yref_e = zeros(16, 1);

ocp.constraints.idxbu = (0:11).';
ocp.constraints.lbu = fullBaseNmpc.uMin(:);
ocp.constraints.ubu = fullBaseNmpc.uMax(:);
ocp.constraints.lh = -1e9*ones(4, 1);
ocp.constraints.uh = zeros(4, 1);
ocp.constraints.idxbx = (12:15).';
ocp.constraints.lbx = [repmat(fullBaseNmpc.xiMin, 2, 1); ...
    repmat(-fullBaseNmpc.dxiMax, 2, 1)];
ocp.constraints.ubx = [repmat(fullBaseNmpc.xiMax, 2, 1); ...
    repmat(fullBaseNmpc.dxiMax, 2, 1)];
ocp.constraints.idxbx_e = ocp.constraints.idxbx;
ocp.constraints.lbx_e = ocp.constraints.lbx;
ocp.constraints.ubx_e = ocp.constraints.ubx;
ocp.constraints.x0 = fullBaseNmpc.model.xEq(:);

ocp.solver_options.N_horizon = fullBaseNmpc.N;
ocp.solver_options.tf = fullBaseNmpc.N*fullBaseNmpc.Ts;
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
ocp.solver_options.integrator_type = 'ERK';
ocp.solver_options.nlp_solver_type = 'SQP_RTI';

simulinkOpts = AcadosOcpSimulinkOptions();
simulinkOpts.inputs = setAllFields(simulinkOpts.inputs, 0);
simulinkOpts.inputs.lbx_0 = 1;
simulinkOpts.inputs.ubx_0 = 1;
simulinkOpts.inputs.y_ref_0 = 1;
simulinkOpts.inputs.y_ref = 1;
simulinkOpts.inputs.y_ref_e = 1;
simulinkOpts.outputs = setAllFields(simulinkOpts.outputs, 0);
simulinkOpts.outputs.u0 = 1;
simulinkOpts.outputs.solver_status = 1;
simulinkOpts.outputs.CPU_time = 1;
simulinkOpts.samplingtime = 't0';
simulinkOpts.show_port_info = 1;
ocp.simulink_opts = simulinkOpts;
ocp.code_gen_options.code_export_directory = char(fullBaseNmpc.generatedDir);
end

function obj = setAllFields(obj, value)
names = properties(obj);
for i = 1:numel(names)
    obj.(names{i}) = value;
end
end
