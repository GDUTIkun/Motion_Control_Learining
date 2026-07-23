%% 2D 轮-无质量摆杆-机体系统的状态空间分析
% 本脚本负责直立静态平衡点附近的线性化状态空间分析。
%
% 动力学建模脚本：
%   derive_wheel_pendulum_body_3dof.m
%
% 这里使用动力学建模得到的标准形式：
%   M(q)*ddq + h(q,dq) + G(q) = Bu*u
%
% 广义坐标：
%   q = [x; theta; phi]
%
% 线性化状态：
%   X = [delta_x;
%        delta_theta;
%        delta_phi;
%        delta_dx;
%        delta_dtheta;
%        delta_dphi]
%
% 输入：
%   U = [delta_tau_w;
%        delta_tau_h]
%
% 需要 MATLAB Symbolic Math Toolbox。数值分析部分不依赖 Control System
% Toolbox；能控矩阵在本脚本中手动构造。

clear;
clc;
close all;

%% 1. 定义符号参数
syms mw mb Iw Ib L r g positive

%% 2. 构造直立静态平衡点附近的状态空间方程
% 定义轮部等效平动质量和总等效质量。
me = simplify(mw + Iw/r^2);
mt = simplify(mb + me);
Delta = simplify(mb*L^2*me);

A_lin = sym(zeros(6, 6));
A_lin(1, 4) = 1;
A_lin(2, 5) = 1;
A_lin(3, 6) = 1;
A_lin(4, 2) = simplify(-mb^2*g*L^2/Delta);
A_lin(5, 2) = simplify(mt*mb*g*L/Delta);

B_lin = sym(zeros(6, 2));
B_lin(4, 1) = simplify((mb*L^2/r + mb*L)/Delta);
B_lin(4, 2) = simplify(mb*L/Delta);
B_lin(5, 1) = simplify(-(mb*L/r + mt)/Delta);
B_lin(5, 2) = simplify(-mt/Delta);
B_lin(6, 2) = simplify(-1/Ib);

disp('==============================================================');
disp('状态空间模型：dX = A_lin*X + B_lin*U');

disp('A_lin =');
disp(A_lin);

disp('B_lin =');
disp(B_lin);

%% 3. 数值参数
% 这里的参数只用于演示系统分析流程。实际分析时替换为真实机器人参数。
params = struct( ...
    'mw', 2.0, ...
    'mb', 8.0, ...
    'Iw', 0.03, ...
    'Ib', 0.60, ...
    'L', 0.50, ...
    'r', 0.10, ...
    'g', 9.81);

[A_num, B_num] = wheelPendulumBody2DLinearMatrices(params);

disp('数值参数 params =');
disp(params);

disp('A_num =');
disp(A_num);

disp('B_num =');
disp(B_num);

%% 4. 特征值和特征向量分析
% eig(A) 给出开环自然模态。特征值决定增长、衰减或振荡特性；
% 特征向量描述该模态中各状态共同变化的比例。
[V_eig, D_eig] = eig(A_num);
lambda = diag(D_eig);

[~, sort_idx] = sort(real(lambda), 'descend');
lambda_sorted = lambda(sort_idx);
V_sorted = V_eig(:, sort_idx);
V_normalized = normalizeEigenvectorsByMaxAbs(V_sorted);

eig_table = table((1:numel(lambda_sorted)).', real(lambda_sorted), imag(lambda_sorted), ...
    'VariableNames', {'mode', 'real_lambda', 'imag_lambda'});

disp('开环特征值（按实部从大到小排序）=');
disp(eig_table);

disp('归一化特征向量列 V_normalized =');
disp(V_normalized);

sigma_star = real(lambda_sorted(1));
lambda_star = lambda_sorted(1);
if sigma_star > 0
    t_double = log(2)/sigma_star;
    fprintf('最危险模态 lambda_star = %.6g%+.6gi\n', real(lambda_star), imag(lambda_star));
    fprintf('该模态翻倍时间 t2 = %.6g s\n', t_double);
else
    disp('未发现正实部特征值，开环没有指数发散模态。');
end

%% 5. 能控性判断
% 能控矩阵：
%   Co = [B, A*B, A^2*B, ..., A^(n-1)*B]
n_state = size(A_num, 1);
Co = controllabilityMatrix(A_num, B_num);
rank_Co = rank(Co);
singular_values_Co = svd(Co);

disp('能控矩阵 Co =');
disp(Co);

fprintf('rank(Co) = %d, state dimension n = %d\n', rank_Co, n_state);
disp('能控矩阵奇异值 svd(Co) =');
disp(singular_values_Co);

if rank_Co == n_state
    disp('结论：该线性化系统在当前数值参数下完全能控。');
else
    disp('结论：该线性化系统在当前数值参数下不完全能控。');
end

%% 6. 数值仿真：非线性模型与线性模型的小扰动对比
% 该检查用于辅助验证线性化模型。一阶模型正确时，小扰动短时间内
% 应与非线性模型响应接近。
u_zero = @(t) [0; 0];
t_span = [0, 0.35];
X0 = [0; deg2rad(1.0); 0; 0; 0; 0];

ode_opts = odeset('RelTol', 1e-9, 'AbsTol', 1e-11);
[t_nl, X_nl] = ode45(@(t, X) wheelPendulumBody2DNonlinearState(t, X, u_zero, params), ...
                     t_span, X0, ode_opts);
[t_li, X_li] = ode45(@(t, X) wheelPendulumBody2DLinearState(t, X, u_zero, ...
                     A_num, B_num), t_span, X0, ode_opts);

X_li_on_nl_grid = interp1(t_li, X_li, t_nl, 'pchip');
max_abs_error = max(abs(X_nl - X_li_on_nl_grid), [], 1);

disp('非线性模型与线性模型各状态最大绝对误差 =');
disp(array2table(max_abs_error, ...
    'VariableNames', {'x', 'theta', 'phi', 'dx', 'dtheta', 'dphi'}));

fig = figure('Name', '2D linearization check', 'Color', 'w');
state_names = {'x', '\theta', '\phi', 'dx', 'd\theta', 'd\phi'};
tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

for k = 1:6
    nexttile;
    plot(t_nl, X_nl(:, k), 'LineWidth', 1.4);
    hold on;
    plot(t_li, X_li(:, k), '--', 'LineWidth', 1.2);
    grid on;
    xlabel('t / s');
    ylabel(state_names{k}, 'Interpreter', 'tex');
    if k == 1
        legend('nonlinear', 'linear', 'Location', 'best');
    end
end

out_dir = fullfile(fileparts(mfilename('fullpath')), 'out');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

linearization_check_path = fullfile(out_dir, 'linearization_check_2d.png');
exportgraphics(fig, linearization_check_path, 'Resolution', 200);
disp(['线性化检查图已保存到：', linearization_check_path]);
disp('==============================================================');

%% 局部函数
function [A_num, B_num] = wheelPendulumBody2DLinearMatrices(p)
%WHEELPENDULUMBODY2DLINEARMATRICES Build numeric A and B matrices.
me = p.mw + p.Iw/p.r^2;
mt = p.mb + me;
Delta = p.mb*p.L^2*me;

A_num = zeros(6, 6);
A_num(1, 4) = 1;
A_num(2, 5) = 1;
A_num(3, 6) = 1;
A_num(4, 2) = -p.mb^2*p.g*p.L^2/Delta;
A_num(5, 2) = mt*p.mb*p.g*p.L/Delta;

B_num = zeros(6, 2);
B_num(4, 1) = (p.mb*p.L^2/p.r + p.mb*p.L)/Delta;
B_num(4, 2) = p.mb*p.L/Delta;
B_num(5, 1) = -(p.mb*p.L/p.r + mt)/Delta;
B_num(5, 2) = -mt/Delta;
B_num(6, 2) = -1/p.Ib;
end

function Co = controllabilityMatrix(A_num, B_num)
%CONTROLLABILITYMATRIX Build [B, AB, ..., A^(n-1)B] without toolboxes.
n_state = size(A_num, 1);
Co = B_num;
A_power = eye(n_state);

for k = 1:(n_state - 1)
    A_power = A_power*A_num;
    Co = [Co, A_power*B_num]; %#ok<AGROW>
end
end

function V_norm = normalizeEigenvectorsByMaxAbs(V)
%NORMALIZEEIGENVECTORSBYMAXABS Scale each eigenvector by its largest entry.
V_norm = V;
for k = 1:size(V, 2)
    [~, idx] = max(abs(V(:, k)));
    if abs(V(idx, k)) > 0
        V_norm(:, k) = V(:, k)/V(idx, k);
    end
end
end

function dX = wheelPendulumBody2DLinearState(t, X, u_fun, A_num, B_num)
%WHEELPENDULUMBODY2DLINEARSTATE Linearized state equation.
u = u_fun(t);
dX = A_num*X + B_num*u;
end

function dX = wheelPendulumBody2DNonlinearState(t, X, u_fun, p)
%WHEELPENDULUMBODY2DNONLINEARSTATE Nonlinear model in first-order form.
theta = X(2);
dx = X(4);
dtheta = X(5);
dphi = X(6); %#ok<NASGU>
u = u_fun(t);

M_num = [p.mw + p.mb + p.Iw/p.r^2, p.mb*p.L*cos(theta), 0;
         p.mb*p.L*cos(theta),       p.mb*p.L^2,          0;
         0,                         0,                   p.Ib];

h_num = [-p.mb*p.L*sin(theta)*dtheta^2;
          0;
          0];

G_num = [0;
        -p.mb*p.g*p.L*sin(theta);
          0];

Bu_num = [1/p.r, 0;
          -1,   -1;
           0,   -1];

ddq_num = M_num \ (Bu_num*u - h_num - G_num);
dX = [dx; dtheta; X(6); ddq_num];
end
