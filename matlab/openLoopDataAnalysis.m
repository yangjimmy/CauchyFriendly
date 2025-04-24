clc; clear; close all;

%%
% load('Rotor_openloop_1020_2.mat');
% load('.\data\Rotor_openloop_ID.mat');
load('.\data\Rotor_openloop_ID_3.mat');


%% Given system parameters
m   = 3.937*1E-2;       % (kg) Mass of the rotor
g   = 9.81;           % (m/s^2) Gravitational acceleration
l_c = 2.54*1E-2;      % (m)Distance from pivot joint to the center of pendulum rod
J_rodc  = 2.12*1E-5;  % (kgm^2) Moment of inertia of pendulum about center of rod
J_motor = 1.67*1E-6;  % (kgm^2)  Moment of inertia of motor rotor
J_rotor = J_rodc + J_motor;     % (kgm^2)Moment of inertia of inertia mode of system
J_dp = J_rotor + m*l_c^2 ;      % 3.7494e-05 %8.54*1E-5;   %(kgm^2) Moment of inertia of downward pendulum mode of system
J_ip = J_dp;          % (kgm^2)Moment of inertia of inverted pendulum mode of system
b = 4.00E-6;          % Motor-rotor viscous damping
R = 3.85;             % (Ohms)Motor coil resistant
L = 0.23e-3;          % (Henry) Motor coil inductance
K_m = 2.40E-2 ;       % (Nm/A) Motor torque constant
V_s = 12-0.65*2  ;    % (V)Supply voltage of the motor drive (H-bridge)
encoder_step =2*pi/400;   % encoder quantization step size



%% Define pseudo system


kappa = 443;
tau = 0.16;
G = tf([kappa,],[tau,1]);

Km_id = J_rotor * R * kappa / V_s / tau;
b_id = Km_id/R*(V_s/kappa-Km_id);

%%

v_sim = lsim(G,u,t);

index = 14580:16580;
% index = 1:length(t);
index = 23000: 31000;

figure(1);
plot(t(index)-t(index(1)),v(index) / (-5),'LineWidth',2); hold on;
plot(t(index)-t(index(1)),v_sim(index),'--','LineWidth',2); hold off;
grid on; grid minor; axis auto; axis tight;
xlabel('Time (s)', 'Interpreter','latex');
ylabel('Velocity (rad/s)','Interpreter','latex');
legend('Real plant','Tuned first-order system', ...
    'Interpreter','latex','Location','southeast','FontSize',10);
title('Open loop step response on real plant and tuned system','Interpreter','latex');
% exportgraphics(gcf,'.\Fig\openLoopID.png','Resolution',600);

%%
K_m = J_rotor * R / V_s * kappa / tau;

b = K_m / R * (V_s / kappa - K_m);

