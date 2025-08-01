% Design a linear quardratic regulator on a linearized pendulum model

% Dynamic equation
% dx_1/dt = x_2
% dx_2/dt = (-m*g*l_c/J)*sin(x_1) - (B/J + K_m^2 / JR)* x_2

% Linearized equation
% x_1 = x_10 + x_1_p
% x_2 = x_20 + x_2_p
% dx_1_p/dt = x_20 + x_2_p
% dx_2_p/dt = (-m*g*l_c/J)*(sin(x_10)+cos(x_10)*x_1_p) 
%             - (B/J + K_m^2 / JR)* (x_20 + x_2_p)

clc; clear; close all;


%% Call pendulum parameter
% generate a `mp` file logging all pendulum parameters
if ~exist('mp','var')
    pendulum_DataFile
end

%% Linearized model

% equilibrium point (x_10, x_20) = (0, 0)
A = [0, 1;
    -mp.m*mp.g*mp.l_c/mp.J, -(mp.B/mp.J + mp.K_m^2/(mp.J*mp.R))];
B = [0; mp.V_s*mp.K_m/mp.R/mp.J;];
Gamma = [0; 1;];
C = [1, 0];

Q = diag([1, 0.1]) * 0.003;
R = 1;

%%
sys = ss(A,B,C,0);
% [K,~] = lqr(sys,Q,R);
sys = c2d(sys,mp.dt);
[K,~] = dlqr(sys.A, sys.B, Q, R)

save('lqr_K',"K");

function dx_dt = linearized_motor_pend_ode(x_p, x_0)
    % x_0: equilibrium point
    % x_p: deviated point
    dx_dt = zeros(2, 1);
    % x1 = theta, x2 = omega
    % currently u does not depend on x
    dx_dt(1) = (x_0(2) + x_p(2));
    dx_dt(2) = (-mp.m*mp.g*mp.l_c/mp.J)*(sin(x_0(1)) + cos(x_0(1)) * x_p(1)) ...
                -(mp.B/mp.J + mp.K_m^2/(mp.J*mp.R)) * (x_0(2) + x_p(2));
    
end
