%% 2D floating-base state-space derivation and LQR design demo
% State:
%   X = [x; z; theta; dx; dz; dtheta]
%
% Input:
%   U = [FHx; FHz; MBy]
%
% Dynamics:
%   m*ddx = FHx
%   m*ddz = FHz - m*g
%   Iyy*ddtheta = r_z(theta)*FHx - r_x(theta)*FHz + MBy

clear;
clc;

addpath(fullfile(fileparts(mfilename("fullpath")), "..", "simulate", ...
    "single_wheel_leg"));

base = struct();
base.m = 3.0;
base.Iyy = 0.25;
base.g = 9.81;
base.rHBody = [0; -0.70];
base.thetaEq = 0;
base.xEq = [0; 0; 0; 0; 0; 0];
base.Q = diag([25, 80, 120, 8, 16, 10]);
base.R = diag([1/80^2, 1/140^2, 1/60^2]);
base.forceMax = [140; 140];
base.momentMax = 160;

model = floating_base_state_space(base);
disp("A =");
disp(model.A);
disp("B =");
disp(model.B);
disp("equilibrium input [FHx; FHz; MBy] =");
disp(model.uEq);

Co = controllabilityMatrix(model.A, model.B);
fprintf("rank(Co) = %d, state dimension = %d\n", ...
    rank(Co), size(model.A, 1));

baseLqr = floating_base_lqr_design(base);
disp("LQR K =");
disp(baseLqr.K);
disp("closed-loop poles =");
disp(baseLqr.poles);

function Co = controllabilityMatrix(A, B)
n = size(A, 1);
Co = B;
Apow = eye(n);
for k = 1:(n - 1)
    Apow = Apow * A;
    Co = [Co, Apow * B]; %#ok<AGROW>
end
end
