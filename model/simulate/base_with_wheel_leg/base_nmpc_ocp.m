function ocp = base_nmpc_ocp(base, leg, baseNmpc)
%BASE_NMPC_OCP Build the planar 8-state base-wheel acados OCP.

arguments
    base (1, 1) struct
    leg (1, 1) struct
    baseNmpc (1, 1) struct
end

import casadi.*

x = SX.sym('x', 8, 1);
u = SX.sym('u', 3, 1);
xdot = SX.sym('xdot', 8, 1);

theta = x(3);
rHBody = base.rHBody(:);
rHx = cos(theta)*rHBody(1) - sin(theta)*rHBody(2);
rHz = sin(theta)*rHBody(1) + cos(theta)*rHBody(2);

rollingDenominator = leg.mw * leg.r + leg.Iw / leg.r;
xiAcceleration = -u(1) / base.m ...
    - (u(1) * leg.r + u(3)) / rollingDenominator;
fExpl = [
    x(4:6);
    u(1) / base.m;
    u(2) / base.m - base.g;
    (rHx*u(2) - rHz*u(1) + u(3)) / base.Iyy;
    x(8);
    xiAcceleration
];

model = AcadosModel();
model.name = 'base_wheel_leg_body';
model.x = x;
model.u = u;
model.xdot = xdot;
model.f_expl_expr = fExpl;
model.f_impl_expr = xdot - fExpl;

stageCost = vertcat(x, u);
W = blkdiag(baseNmpc.Q, baseNmpc.R);

ocp = AcadosOcp();
ocp.name = char(baseNmpc.solverName);
ocp.model = model;

ocp.cost.cost_type_0 = 'NONLINEAR_LS';
ocp.model.cost_y_expr_0 = stageCost;
ocp.cost.W_0 = W;
ocp.cost.yref_0 = zeros(11, 1);

ocp.cost.cost_type = 'NONLINEAR_LS';
ocp.model.cost_y_expr = stageCost;
ocp.cost.W = W;
ocp.cost.yref = zeros(11, 1);

ocp.cost.cost_type_e = 'NONLINEAR_LS';
ocp.model.cost_y_expr_e = x;
ocp.cost.W_e = baseNmpc.W_e;
ocp.cost.yref_e = zeros(8, 1);

ocp.constraints.idxbu = (0:2).';
ocp.constraints.lbu = baseNmpc.uMin(:);
ocp.constraints.ubu = baseNmpc.uMax(:);
ocp.constraints.idxbx = [6; 7];
ocp.constraints.lbx = [baseNmpc.xiMin; -baseNmpc.dxiMax];
ocp.constraints.ubx = [baseNmpc.xiMax;  baseNmpc.dxiMax];
ocp.constraints.idxbx_e = [6; 7];
ocp.constraints.lbx_e = ocp.constraints.lbx;
ocp.constraints.ubx_e = ocp.constraints.ubx;
ocp.constraints.x0 = baseNmpc.model.xEq(:);

ocp.solver_options.N_horizon = baseNmpc.N;
ocp.solver_options.tf = baseNmpc.N * baseNmpc.Ts;
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

ocp.code_gen_options.code_export_directory = char(baseNmpc.generatedDir);
end

function obj = setAllFields(obj, value)
names = properties(obj);
for i = 1:numel(names)
    obj.(names{i}) = value;
end
end
