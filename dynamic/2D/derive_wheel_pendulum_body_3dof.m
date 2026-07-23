%% 三广义坐标轮-无质量摆杆-机体系统的拉格朗日动力学推导
% 广义坐标：q = [x; theta; phi]
%
% x     ：轮子质心沿水平向右的位移，向右为正；
% theta ：摆杆相对竖直方向的倾角，顺时针为正；
% phi   ：机体绝对俯仰角，逆时针为正。
%
% 输入：u = [tau_w; tau_h]
%
% tau_w ：轮部电机施加在轮子上的顺时针力矩，顺时针为正；
% tau_h ：髋部电机施加在机体上的顺时针力矩，顺时针为正。
%
% 结构和建模假设：
% 1. 摆杆为长度 L 的理想无质量刚性杆，质量和转动惯量均为 0；
% 2. 轮子质心与机体质心分别位于摆杆两端，因此它们到摆杆中点
%    的距离均为 L/2，而两质心之间的距离为 L；
% 3. 轮子在水平地面上纯滚动，无滑动，因此轮子顺时针转角
%    alpha_w = x/r，顺时针角速度 omega_w = dx/r；
% 4. 机体质心位于摆杆上端，但机体可以通过髋部电机相对摆杆转动；
% 5. 不考虑摩擦、阻尼、轮胎变形和摆杆质量；
% 6. mw、Iw 表示等效轮部总质量和总转动惯量。如果实际有两个完全
%    相同且同步运动的轮子，可令 mw=2*mw_single、Iw=2*Iw_single。
%
% 最终推导形式：
%       M(q)*ddq + h(q,dq) + G(q) = Bu*u
%
% 需要 MATLAB Symbolic Math Toolbox。

clear;
clc;

%% 1. 定义广义坐标、速度、加速度和物理参数
syms x theta phi real
syms dx dtheta dphi real
syms ddx ddtheta ddphi real
syms mw mb Iw Ib L r g positive
syms tau_w tau_h real

q   = [x; theta; phi];
dq  = [dx; dtheta; dphi];
ddq = [ddx; ddtheta; ddphi];
u   = [tau_w; tau_h];

%% 2. 建立轮子质心和机体质心的位置
% 取水平向右为 X 正方向、竖直向上为 Y 正方向。
% 轮子始终接触水平地面，因此轮心高度恒为 r。
p_w = [x;
       r];

% theta 从竖直方向顺时针为正，所以：
% 水平投影为  L*sin(theta)，竖直投影为 L*cos(theta)。
p_b = [x + L*sin(theta);
       r + L*cos(theta)];

%% 3. 由位置雅可比计算线速度
% 对任意位置 p(q)，都有 v = J(q)*dq，其中 J = partial(p)/partial(q)。
J_w = jacobian(p_w, q);
J_b = jacobian(p_b, q);

v_w = simplify(J_w*dq);
v_b = simplify(J_b*dq);

% 结果应为：
% v_w = [dx; 0]
% v_b = [dx + L*cos(theta)*dtheta;
%       -L*sin(theta)*dtheta]

%% 4. 建立各部分动能
% 4.1 轮子平动动能
T_w_trans = simplify(1/2*mw*(v_w.'*v_w));

% 4.2 轮子转动动能
% 纯滚动：轮子顺时针角速度 omega_w = dx/r。
omega_w = dx/r;
T_w_rot = simplify(1/2*Iw*omega_w^2);

% 4.3 理想无质量摆杆动能
% 摆杆质量和转动惯量均为零，因此不储存动能。
T_rod = sym(0);

% 4.4 机体平动动能
T_b_trans = simplify(1/2*mb*(v_b.'*v_b));

% 4.5 机体绕自身质心的转动动能
% phi 为机体逆时针绝对俯仰角，因此机体角速度为 dphi。
T_b_rot = simplify(1/2*Ib*dphi^2);

% 系统总动能
T = simplify(T_w_trans + T_w_rot + T_rod + T_b_trans + T_b_rot);

%% 5. 建立势能
% 轮子质心高度 r 为常数，轮子势能 mw*g*r 不影响动力学，可删除。
% 无质量摆杆的势能为零。
% 机体质心高度为 r + L*cos(theta)。删除常数 mb*g*r 后：
V_w   = sym(0);
V_rod = sym(0);
V_b   = mb*g*L*cos(theta);

V = simplify(V_w + V_rod + V_b);

%% 6. 构造拉格朗日量
Lag = simplify(T - V);

%% 7. 用执行器相对转角推导广义力
% 广义力由虚功定义：delta_W = Q.'*delta_q。
%
% 7.1 轮部电机
% 轮子顺时针绝对转角 alpha_w = x/r；摆杆顺时针角为 theta。
% 轮部电机的顺时针相对转角为：
%       eta_w = alpha_w - theta = x/r - theta
% 因此：
%       delta_W_w = tau_w*delta_eta_w
%                 = (tau_w/r)*delta_x - tau_w*delta_theta。
eta_w = x/r - theta;

% 7.2 髋部电机
% 机体的坐标 phi 规定逆时针为正，所以机体顺时针角为 -phi。
% 摆杆顺时针角为 theta，因此“机体相对摆杆”的顺时针转角为：
%       eta_h = (-phi) - theta = -phi - theta
% 因此髋部电机对机体施加顺时针正力矩 tau_h 时：
%       delta_W_h = tau_h*delta_eta_h
%                 = -tau_h*delta_theta - tau_h*delta_phi。
% 这也反映了：机体受到顺时针力矩，而摆杆受到大小相等、方向相反
% 的物理反作用力矩；由于 theta 的正方向本身是顺时针，摆杆上的
% 逆时针反作用力矩在 theta 坐标中同样表现为负号。
eta_h = -phi - theta;

eta = [eta_w; eta_h];

% 若执行器相对坐标为 eta(q)，则：
%       delta_W = u.'*delta_eta
%               = u.'*(partial eta/partial q)*delta_q
%               = Q.'*delta_q
% 所以 Q = (partial eta/partial q).'*u。
J_eta = jacobian(eta, q);
Bu    = simplify(J_eta.');
Q     = simplify(Bu*u);

% 推导结果应为：
%             [ 1/r    0 ]
%       Bu =  [ -1    -1 ]
%             [  0    -1 ]
%
%             [  tau_w/r       ]
%       Q  =  [ -tau_w - tau_h ]
%             [ -tau_h         ]

%% 8. 自动构造 Euler-Lagrange 方程
% 对每个广义坐标 qi：
%       d/dt(partial Lag/partial dqi) - partial Lag/partial qi = Qi
%
% 由于这里把 q、dq、ddq 定义为相互独立的符号，时间全导数使用链式法则：
%       d/dt(f(q,dq)) = partial(f)/partial(q)*dq
%                     + partial(f)/partial(dq)*ddq。
dLag_ddq = jacobian(Lag, dq).';
dLag_dq  = jacobian(Lag, q).';

dt_dLag_ddq = jacobian(dLag_ddq, q)*dq ...
             + jacobian(dLag_ddq, dq)*ddq;

EL = simplify(dt_dLag_ddq - dLag_dq);

% 非线性动力学方程：EL = Q
EOM = EL == Q;

%% 9. 提取 M(q)、h(q,dq)、G(q) 和输入矩阵 Bu
% EL 对 ddq 是线性的，其系数即质量矩阵 M(q)。
M = simplify(jacobian(EL, ddq));

% 去掉 M*ddq 后，剩余项包含速度非线性项和重力项。
remaining = simplify(EL - M*ddq);

% 将所有广义速度置零，得到重力项 G(q)。
G = simplify(subs(remaining, dq, sym(zeros(3,1))));

% 剩余部分为速度非线性项 h(q,dq)。
h = simplify(remaining - G);

% 最终标准形式：
%       M*ddq + h + G = Bu*u
EOM_matrix = M*ddq + h + G == Bu*u;

%% 10. 构造一个满足 C(q,dq)*dq = h(q,dq) 的 C 矩阵
% C 矩阵的写法并不唯一；这里选取最简单的一种。
C = sym(zeros(3));
C(1,2) = -mb*L*sin(theta)*dtheta;
C = simplify(C);

% 验证 C*dq 与自动提取的 h 完全一致。
C_check = simplify(C*dq - h);

%% 11. 给出广义加速度的显式表达式
% 在 M 非奇异的工作区间内：ddq = M^(-1)*(Bu*u - h - G)。
ddq_explicit = simplify(M \ (Bu*u - h - G));

%% 12. 与人工预期结果进行符号校验
A = mw + mb + Iw/r^2;

M_expected = [A,                 mb*L*cos(theta), 0;
              mb*L*cos(theta),  mb*L^2,          0;
              0,                 0,                Ib];

h_expected = [-mb*L*sin(theta)*dtheta^2;
               0;
               0];

G_expected = [0;
             -mb*g*L*sin(theta);
               0];

Bu_expected = [1/r,  0;
               -1,  -1;
                0,  -1];

check_M  = simplify(M  - M_expected);
check_h  = simplify(h  - h_expected);
check_G  = simplify(G  - G_expected);
check_Bu = simplify(Bu - Bu_expected);

%% 13. 在命令窗口显示推导结果
disp('==============================================================');
disp('广义坐标 q =');
disp(q);

disp('总动能 T =');
pretty(T);

disp('总势能 V =');
pretty(V);

disp('拉格朗日量 Lag = T - V =');
pretty(Lag);

disp('输入矩阵 Bu =');
disp(Bu);

disp('广义力 Q = Bu*u =');
disp(Q);

disp('质量矩阵 M(q) =');
disp(M);

disp('速度非线性项 h(q,dq) =');
disp(h);

disp('重力项 G(q) =');
disp(G);

disp('一个满足 C*dq = h 的 C(q,dq) =');
disp(C);

disp('Euler-Lagrange 方程 EL = Q：');
disp(EOM);

disp('矩阵形式 M*ddq + h + G = Bu*u：');
disp(EOM_matrix);

disp('显式广义加速度 ddq =');
disp(ddq_explicit);

disp('符号校验结果（以下矩阵均应为全零）：');
disp('check_M =');  disp(check_M);
disp('check_h =');  disp(check_h);
disp('check_G =');  disp(check_G);
disp('check_Bu ='); disp(check_Bu);
disp('C*dq - h ='); disp(C_check);
disp('==============================================================');

%% 14. 最终三条标量动力学方程（便于人工核对）
% 令 A = mw + mb + Iw/r^2，则：
%
% (1) x 方向：
%     A*ddx + mb*L*cos(theta)*ddtheta
%     - mb*L*sin(theta)*dtheta^2 = tau_w/r
%
% (2) theta 方向（theta 顺时针为正）：
%     mb*L*cos(theta)*ddx + mb*L^2*ddtheta
%     - mb*g*L*sin(theta) = -tau_w - tau_h
%
% (3) phi 方向（phi 逆时针为正）：
%     Ib*ddphi = -tau_h
%
% 即：
%                      [ 1/r    0 ] [tau_w]
%     M*ddq + h + G =  [ -1    -1 ] [tau_h]
%                      [  0    -1 ]
