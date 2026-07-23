%% Minimal 3D wheel-leg inverted-pendulum model with variable leg length
% 最简三维轮腿倒立摆符号建模过程（约束消元版）
%
% 本脚本的目标：
%   1. 沿用前面讨论得到的最简广义坐标；
%   2. 不把 y、alpha_L、alpha_R 作为独立广义坐标；
%   3. 直接把双轮纯滚动关系代入轮子速度和虚功；
%   4. 自动从 T-V 提取 M(q)、h(q,dq)、G(q)、Bu。
%
% 广义坐标：
%   q = [x; psi; theta; beta; rho; l]
%
% 变量含义：
%   x     : 机器人沿自身前进方向的滚动位移，不是世界系 X 坐标
%   psi   : 轮轴中点系 A 相对世界系 W 的偏航角，绕 +Z 轴为正
%   theta : 摆杆相对 A_z 的 pitch 角，绕 +Y_A 为正
%   beta  : 机体 B 相对 A 的 pitch 角，绕 +Y_A 为正
%   rho   : 机体 B 相对 A 的 roll 角，绕 +X 轴为正
%   l     : 轮轴中点 A 到髋部/机体质心 B 的距离，可变腿长
%
% 最重要的建模选择：
%   - 轮轴中点系 A 的原点在两轮轴心中点。
%   - 世界系 W 固定在地面，初始时 W 与 A 重合。
%   - 机体系 B 的原点取在髋部，也就是机体质心。
%   - 腿无质量，腿只通过 theta 和 l 决定机体质心位置。
%   - 双轮纯滚动、无侧滑已经被消元，所以最终方程中没有 Jc^T*lambda。
%
% 最终形式：
%   M(q)*ddq + h(q,dq) + G(q) = Bu*u
%
% 需要 Symbolic Math Toolbox。

clear;
clc;

%% 1. 定义广义坐标、速度、加速度和物理参数
syms x psi theta beta rho l real
syms dx dpsi dtheta dbeta drho dl real
syms ddx ddpsi ddtheta ddbeta ddrho ddl real

% 每个轮子的质量为 mw。左右两轮总质量是 2*mw。
syms mw mb real positive

% 机体惯量，默认 B 系选在机体主惯量轴上，所以惯量张量为对角阵。
syms Ibx Iby Ibz real positive

% 轮子惯量：
%   Iwy : 绕轮轴 Y_A 的自转惯量，和 alpha_dot 对应
%   Iwz : 轮子/轮组随底盘偏航时，绕 Z_A 方向的等效转动惯量
% 如果只想保留最简单的轮子自转能量，可在后面令 Iwz = 0。
syms Iwy Iwz real positive

% 结构参数：
%   r : 轮半径
%   d : 轮距，即左右轮轴心之间的距离
%   g : 重力加速度
syms r d g real positive

% 输入：
%   tau_L   : 左轮电机力矩
%   tau_R   : 右轮电机力矩
%   tau_h   : pitch 髋关节力矩，作用在 eta_h = beta - theta 上
%   F_l     : 沿摆杆方向的等效伸缩力，作用在 l 上
%   tau_rho : roll 方向直接力矩；若没有 roll 执行器，可令 tau_rho = 0
syms tau_L tau_R tau_h F_l tau_rho real

q   = [x;  psi;  theta;  beta;  rho;  l];
dq  = [dx; dpsi; dtheta; dbeta; drho; dl];
ddq = [ddx; ddpsi; ddtheta; ddbeta; ddrho; ddl];
u   = [tau_L; tau_R; tau_h; F_l; tau_rho];

%% 2. 基础旋转矩阵
% 记号采用机器人学常用形式：
%   ^W R_A = Rz(psi)
%   ^A R_B = Ry(beta)*Rx(rho)
%   ^W R_B = ^W R_A * ^A R_B
%
% 这里的 beta、rho 只描述机体相对 A 系的姿态；
% theta 只描述摆杆方向，不直接决定机体系 B 的旋转。

Rz_psi = [ cos(psi), -sin(psi), 0;
           sin(psi),  cos(psi), 0;
                  0,         0, 1];

Ry_beta = [ cos(beta), 0, sin(beta);
                   0, 1,         0;
           -sin(beta), 0, cos(beta)];

Rx_rho = [1,        0,         0;
          0, cos(rho), -sin(rho);
          0, sin(rho),  cos(rho)];

R_WA = Rz_psi;
R_AB = Ry_beta*Rx_rho;
R_WB = R_WA*R_AB;

%% 3. 位姿关系：^A p_B 和 ^W R_B
% A 系原点在当前两轮轴心中点。
% B 系原点在髋部/机体质心。
%
% 可变腿长模型中：
%   ^A p_B = [l*sin(theta); 0; l*cos(theta)]

p_B_A = [l*sin(theta);
         0;
         l*cos(theta)];

% 如果需要世界系位置，可以通过积分恢复轮轴中点世界位置：
%   dX = dx*cos(psi)
%   dY = dx*sin(psi)
% 然后：
%   ^W p_B = ^W p_A + ^W R_A * ^A p_B
%
% 符号动力学只需要速度和能量，所以这里直接在 A 系下写速度。

%% 4. 机体质心线速度
% 机体质心 B 的速度由三部分组成：
%   1. A 原点沿 x_A 方向前进：       [dx; 0; 0]
%   2. 腿长和腿角改变引起 p_B_A 变化： d/dt(^A p_B)
%   3. A 系偏航引起的附加速度：       omega_A x p_B_A
%
% 其中 ^A omega_A = [0; 0; dpsi]。

v_A_A = [dx; 0; 0];

dp_B_A = jacobian(p_B_A, [theta, l]) * [dtheta; dl];

omega_A_A = [0; 0; dpsi];
v_yaw_A = cross(omega_A_A, p_B_A);

v_B_A = simplify(v_A_A + dp_B_A + v_yaw_A);

% 展开结果应为：
%   v_B_A =
%       [dx + dl*sin(theta) + l*cos(theta)*dtheta;
%        l*sin(theta)*dpsi;
%        dl*cos(theta) - l*sin(theta)*dtheta]
%
% 世界系速度为 v_B_W = R_WA*v_B_A。
% 因为 R_WA 是正交矩阵，所以 v_B_W.'*v_B_W = v_B_A.'*v_B_A。
v_B_sq = simplify(v_B_A.'*v_B_A);

%% 5. 机体角速度
% 姿态顺序：
%   ^W R_B = Rz(psi)*Ry(beta)*Rx(rho)
%
% 对这个旋转顺序，机体角速度在 W 系下可以写成：
%   ^W omega_B =
%       dpsi  * e_z_W
%     + dbeta * Rz(psi)*e_y
%     + drho  * Rz(psi)*Ry(beta)*e_x

e_x = [1; 0; 0];
e_y = [0; 1; 0];
e_z = [0; 0; 1];

omega_B_W = simplify(dpsi*e_z + dbeta*(Rz_psi*e_y) + drho*(Rz_psi*Ry_beta*e_x));

% 机体惯量通常在 B 系下给出。由于此处使用 W 系角速度，
% 需要把惯量张量也转到 W 系：
%   ^W I_B = ^W R_B * ^B I_B * (^W R_B)^T

I_B_B = diag([Ibx, Iby, Ibz]);
I_B_W = simplify(R_WB*I_B_B*R_WB.');

%% 6. 左右轮速度和纯滚动消元
% 不把 alpha_L、alpha_R 作为广义坐标。
% 对差速双轮，左右轮中心沿 x_A 方向的速度为：
%   v_CL = dx - d/2*dpsi
%   v_CR = dx + d/2*dpsi
%
% 纯滚动关系：
%   dalpha_L = (dx - d/2*dpsi)/r
%   dalpha_R = (dx + d/2*dpsi)/r

v_CL_A = [dx - d/2*dpsi; 0; 0];
v_CR_A = [dx + d/2*dpsi; 0; 0];

dalpha_L = (dx - d/2*dpsi)/r;
dalpha_R = (dx + d/2*dpsi)/r;

% 轮子角速度在 A 系下写成：
%   [0; dalpha; dpsi]
% 第二项是轮子绕轮轴滚动，第三项是整个底盘偏航。

omega_L_A = [0; dalpha_L; dpsi];
omega_R_A = [0; dalpha_R; dpsi];

I_w_A = diag([sym(0), Iwy, Iwz]);

%% 7. 动能
% 7.1 机体平动动能
T_B_trans = simplify(1/2*mb*v_B_sq);

% 7.2 机体转动动能
T_B_rot = simplify(1/2*omega_B_W.'*I_B_W*omega_B_W);

% 7.3 左右轮平动动能
% 注意：每个轮子的质量都是 mw。
T_w_trans = simplify(1/2*mw*(v_CL_A.'*v_CL_A) + 1/2*mw*(v_CR_A.'*v_CR_A));

% 7.4 左右轮转动动能
T_w_rot = simplify(1/2*omega_L_A.'*I_w_A*omega_L_A + ...
                   1/2*omega_R_A.'*I_w_A*omega_R_A);

% 总动能
T = simplify(T_B_trans + T_B_rot + T_w_trans + T_w_rot);

%% 8. 势能
% 零势能面取世界系 x_W O y_W 平面。
% 腿无质量，轮轴高度在当前建模中取为 0。
% 机体质心高度为 l*cos(theta)。

V = mb*g*l*cos(theta);

%% 9. 拉格朗日量
Lag = simplify(T - V);

%% 10. 广义力：由虚功得到
% 输入坐标先定义为执行器的相对位移/角度：
%
%   eta_wL   = (x - d/2*psi)/r - theta
%   eta_wR   = (x + d/2*psi)/r - theta
%   eta_h    = beta - theta
%   eta_l    = l
%   eta_rho  = rho
%
% 虚功：
%   deltaW = tau_L*delta(eta_wL)
%          + tau_R*delta(eta_wR)
%          + tau_h*delta(eta_h)
%          + F_l*delta(l)
%          + tau_rho*delta(rho)
%
% 因此：
%   Q = (partial eta / partial q)^T * u = Bu*u

% The wheel motor is mounted between the wheel and the leg/wheel carrier.
% Therefore its actuator coordinate is the wheel rolling angle relative to
% the leg angle theta. This is what produces the reaction torque on theta
% and makes the planar reduction match the existing 2D model.
eta_wL  = (x - d/2*psi)/r - theta;
eta_wR  = (x + d/2*psi)/r - theta;
eta_h   = beta - theta;
eta_l   = l;
eta_rho = rho;

eta = [eta_wL; eta_wR; eta_h; eta_l; eta_rho];

Bu = simplify(jacobian(eta, q).');
Q  = simplify(Bu*u);

%% 11. Euler-Lagrange 方程
% 对每个 qi：
%   d/dt(partial Lag / partial dqi) - partial Lag / partial qi = Qi
%
% 这里 q、dq、ddq 被定义为互相独立的符号。
% 对 f(q,dq) 求时间全导数时，用链式法则：
%   d/dt f = partial(f)/partial(q)*dq + partial(f)/partial(dq)*ddq

dLag_ddq = jacobian(Lag, dq).';
dLag_dq  = jacobian(Lag, q).';

dt_dLag_ddq = jacobian(dLag_ddq, q)*dq + jacobian(dLag_ddq, dq)*ddq;

EL = simplify(dt_dLag_ddq - dLag_dq);

%% 12. 提取 M(q)、h(q,dq)、G(q)
% EL = M(q)*ddq + h(q,dq) + G(q)
%
% 质量矩阵：
%   M = partial(EL)/partial(ddq)

M = simplify(jacobian(EL, ddq));

% 去掉 M*ddq 后，剩余项包含速度非线性项和重力项。
remaining = simplify(EL - M*ddq);

% 把所有速度置零，剩余就是 G(q)。
G = simplify(subs(remaining, dq, sym(zeros(size(dq)))));

% 余下部分为 h(q,dq)。
h = simplify(remaining - G);

EOM_matrix = M*ddq + h + G == Bu*u;

%% 13. 基本一致性检查
% 1. M 应该是对称矩阵。
% 2. G 应该只来自势能 V。
% 3. Bu 应该反映输入对 q 的虚功映射。

check_M_symmetric = simplify(M - M.');
check_G_from_V    = simplify(G - jacobian(V, q).');

%% 14. 便于阅读的关键中间结果
disp('==============================================================');
disp('Minimal 3D wheel-leg model with variable leg length');
disp('q = [x; psi; theta; beta; rho; l]');
disp('u = [tau_L; tau_R; tau_h; F_l; tau_rho]');
disp('==============================================================');

disp('^A p_B =');
disp(p_B_A);

disp('v_B expressed in A frame =');
disp(v_B_A);

disp('v_B^T*v_B =');
disp(v_B_sq);

disp('omega_B expressed in W frame =');
disp(omega_B_W);

disp('Left and right wheel rolling rates:');
disp('dalpha_L ='); disp(dalpha_L);
disp('dalpha_R ='); disp(dalpha_R);

disp('Total kinetic energy T =');
pretty(T);

disp('Potential energy V =');
pretty(V);

disp('Input mapping Bu =');
disp(Bu);

disp('Generalized force Q = Bu*u =');
disp(Q);

disp('Mass matrix M(q) =');
disp(M);

disp('Velocity nonlinear term h(q,dq) =');
disp(h);

disp('Gravity term G(q) =');
disp(G);

disp('Matrix-form EOM: M*ddq + h + G = Bu*u');
disp(EOM_matrix);

disp('Checks, these should be zero matrices/vectors:');
disp('M - M^T ='); disp(check_M_symmetric);
disp('G - dV/dq ='); disp(check_G_from_V);
disp('==============================================================');

%% 15. 备注：如果要得到最简单的轮子转动模型
% 当前 T_w_rot 包含：
%   1. 轮子绕轮轴的滚动自转惯量 Iwy；
%   2. 轮子随底盘偏航的等效惯量 Iwz。
%
% 如果你想暂时忽略轮子随偏航的转动动能，可以在结果中代入：
%   Iwz = 0
%
% 例如：
%   M_simple = simplify(subs(M, Iwz, 0));
%   h_simple = simplify(subs(h, Iwz, 0));
%
% 如果没有 roll 执行器，可以在输入中令：
%   tau_rho = 0
