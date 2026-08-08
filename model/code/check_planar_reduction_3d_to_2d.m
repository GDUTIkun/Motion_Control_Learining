%% Check planar reduction of the minimal 3D model against the 2D model
% This script does NOT modify either derivation file.
%
% Files being checked conceptually:
%   2D: D:\Workspace\CodeWorkspace\dynamic\2D\derive_wheel_pendulum_body_3dof.m
%   3D: D:\Workspace\CodeWorkspace\dynamic\3D\derive_wheel_leg_body_3d_minimal_variable_l.m
%
% Purpose:
%   Reduce the 3D minimal variable-leg model to planar motion and compare
%   the result with the existing 2D model term by term.
%
% Planar reduction used here:
%   psi = 0, dpsi = 0, ddpsi = 0
%   rho = 0, drho = 0, ddrho = 0
%   l = L, dl = 0, ddl = 0
%   beta = -phi
%   dbeta = -dphi
%   ddbeta = -ddphi
%
% Parameter mapping:
%   2D mw = 2*3D mw_single
%   2D Iw = 2*3D Iwy_single
%   2D Ib = 3D Iby
%   tau_w = tau_L + tau_R, with tau_L = tau_R = tau_w/2 for synchronous rolling

clear;
clc;

%% 1. Symbols
syms x theta phi real
syms dx dtheta dphi real
syms ddx ddtheta ddphi real
syms mw mb Iwy Iby L r g real positive
syms tau_w tau_L tau_R tau_h real

q   = [x; theta; phi];
dq  = [dx; dtheta; dphi];
ddq = [ddx; ddtheta; ddphi];

%% 2. Existing 2D reference model, using total wheel mass and inertia
% In the 2D derivation file, mw and Iw represent total equivalent wheel
% mass and total equivalent wheel inertia. Here the 3D symbol mw is the
% mass of one wheel, and Iwy is the inertia of one wheel around its axle.

mw_total = 2*mw;
Iw_total = 2*Iwy;
Ib_2d    = Iby;

M_2d = [mw_total + mb + Iw_total/r^2, mb*L*cos(theta), 0;
        mb*L*cos(theta),              mb*L^2,          0;
        0,                            0,               Ib_2d];

h_2d = [-mb*L*sin(theta)*dtheta^2;
         0;
         0];

G_2d = [0;
       -mb*g*L*sin(theta);
        0];

Q_2d = [tau_w/r;
       -tau_w - tau_h;
       -tau_h];

%% 3. Planar-reduced 3D energy
% Start from the 3D minimal model after applying:
%   psi = 0, rho = 0, l = L, beta = -phi.
%
% Body COM velocity:
%   v_B^T v_B = dx^2 + 2*L*cos(theta)*dx*dtheta + L^2*dtheta^2
%
% Body angular velocity:
%   beta = -phi  =>  dbeta = -dphi
%   T_body_rot = 1/2*Iby*dphi^2
%
% Wheel energies:
%   two wheels, each mass mw
%   synchronous rolling gives dalpha_L = dalpha_R = dx/r

T_body_trans = 1/2*mb*(dx^2 + 2*L*cos(theta)*dx*dtheta + L^2*dtheta^2);
T_body_rot   = 1/2*Iby*dphi^2;
T_wheel_trans = mw*dx^2;
T_wheel_spin  = Iwy*dx^2/r^2;

T_3d_red = simplify(T_body_trans + T_body_rot + T_wheel_trans + T_wheel_spin);
V_3d_red = mb*g*L*cos(theta);
Lag_3d_red = simplify(T_3d_red - V_3d_red);

%% 4. Extract M, h, G from the reduced 3D Lagrangian
dLag_ddq = jacobian(Lag_3d_red, dq).';
dLag_dq  = jacobian(Lag_3d_red, q).';

dt_dLag_ddq = jacobian(dLag_ddq, q)*dq + jacobian(dLag_ddq, dq)*ddq;
EL_3d_red = simplify(dt_dLag_ddq - dLag_dq);

M_3d_red = simplify(jacobian(EL_3d_red, ddq));
remaining_3d_red = simplify(EL_3d_red - M_3d_red*ddq);
G_3d_red = simplify(subs(remaining_3d_red, dq, sym(zeros(size(dq)))));
h_3d_red = simplify(remaining_3d_red - G_3d_red);

check_M = simplify(M_3d_red - M_2d);
check_h = simplify(h_3d_red - h_2d);
check_G = simplify(G_3d_red - G_2d);

%% 5. Input mapping check
% Current 3D file uses the wheel angle relative to the leg:
%   eta_wL = (x - d/2*psi)/r - theta
%   eta_wR = (x + d/2*psi)/r - theta
%   eta_h  = beta - theta
%
% After planar reduction beta = -phi and tau_L + tau_R = tau_w:
%   Q_2d = [tau_w/r; -tau_w - tau_h; -tau_h]

Q_3d_current_red = [tau_w/r;
                   -tau_w - tau_h;
                   -tau_h];

check_Q_current = simplify(Q_3d_current_red - Q_2d);

%% 6. Display
disp('==============================================================');
disp('Planar reduction check: 3D minimal model -> existing 2D model');
disp('==============================================================');

disp('M_3d_reduced - M_2d =');
disp(check_M);

disp('h_3d_reduced - h_2d =');
disp(check_h);

disp('G_3d_reduced - G_2d =');
disp(check_G);

disp('Current 3D input map after reduction minus 2D input map =');
disp(check_Q_current);

disp('Conclusion:');
if isequal(check_M, sym(zeros(3))) && ...
   isequal(check_h, sym(zeros(3,1))) && ...
   isequal(check_G, sym(zeros(3,1)))
    disp('  Energy terms M, h, and G reduce exactly to the 2D model.');
else
    disp('  Energy terms do NOT reduce exactly to the 2D model.');
end

if isequal(check_Q_current, sym(zeros(3,1)))
    disp('  Current wheel-relative 3D input map also reduces exactly to the 2D input map.');
else
    disp('  Current 3D input map does NOT reduce to the 2D input map.');
end

disp('==============================================================');
