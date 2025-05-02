clear
clc
close all

global mp
rmpath("nl_tut_callbacks")
addpath("matlab_pure");
addpath("mp_callbacks");
addpath("mex_files");
addpath(pwd);

% pendulum configuration
mp = struct(...
    'g', 9.81, ... % gravitational constant
    'B', 8.1055e-6, ... % motor damping
    'L', 0.23e-3, ... % inductance
    'R', 3.85, ... % resistance
    'V_s', 10.7, ... % supply voltage
    'K_m', 0.0228, ... % motor constant
    'm', 0.03937, ... % mass of rod
    'l_c', 0.0254, ... % length of rod
    'J_motor', 1.67e-6, ...
    'J_rod', 2.12*1E-5, ...
    'sr', 1000, ... % sampling rate (Hz) % 100
    'w_PSD', 0.01 ... % process noise Power spectral density
);
% mp.J_rod = 1/12*mp.m; % *mp.l_c^2; % pendulum config; 1/12*m*l_c^2 for pendulum config
mp.J = mp.J_motor + mp.J_rod + mp.m * mp.l_c^2;
mp.dt = 1/mp.sr;

% %% Bode Transfer Function
% s = tf('s');
% basic_tf = (mp.K_m*mp.V_s/(mp.J*mp.R))/(s^2+s*(mp.B/mp.J+mp.K_m^2/(mp.J*mp.R)));
% bode(basic_tf);

% TODO: pendulum colliding with wall

%% Generate a trajectory
theta_vec0 = [pi/2; 0]; % initial angle of 45 degrees at 0 radians/sec
theta_k = theta_vec0;
thetas = theta_k';
% propagations = 160;
propagations = 4000;


for k = 1:propagations
    theta_k = nonlin_transition_model(theta_k);
    thetas = [thetas; theta_k'];
end
Ts = ((0:propagations) * mp.dt)';
figure;
sgtitle('Pendulum Trajectory (angle: top), (angular rate: bottom)');
subplot(2, 1, 1);
plot(Ts, thetas(:, 1));
subplot(2, 1, 2);
plot(Ts, thetas(:, 2));

% plot makes sense since back EMF prevents the motor from rotating 

%% Generate trajectory with noise
% Creating the dynamic simulation
V = 0.009311^2; % best fit Gaussian
H = [1.0, 0.0]; % meausrement model
xk = theta_vec0;
xs = xk'; % State vector history
ws = [];   % Process noise history
vs = unifrnd(-pi/200,pi/200); % Measurement noise history; sample from uniform distribution
zs = H * xk + vs(1); % Measurement history
% propagations = 160;

for k = 1:propagations
    wk = mp.dt * sqrt(mp.w_PSD) * randn();
    xk(2) = xk(2) + wk; % Gamma = [0; 1];
    xk = nonlin_transition_model(xk);
    xs = [xs; xk'];
    ws = [ws; wk];
    vk = unifrnd(-pi/200,pi/200); % sample from uniform distribution
    zk = H * xk + vk;
    vs = [vs; vk];
    zs = [zs; zk];
end
plot_simulation_history([], {xs,zs,ws,vs}, [])

%% Kalman Filter
% Continuous time Gamma (\Gamma_c)
Gamma_c = [0.0; 1.0];
W_c = mp.w_PSD;
I2 = eye(2);
taylor_order = 2;

% Setting up and running the EKF
% The gaussian_filters module has a "run_ekf" function baked in, but we'll just show the whole thing here
P0_kf = eye(2) * 0.003;
x0_kf = mvnrnd(theta_vec0, P0_kf); % lets initialize the Kalman filter slightly off from the true state position

xs_kf = x0_kf;
Ps_kf = zeros(propagations+1, 2, 2);
Ps_kf(1, :, :) = P0_kf;
x_kf = x0_kf';
P_kf = P0_kf;
for k = 1:propagations
    Jac_F = jacobian_mp_ode(x_kf);
    [Phi_k, W_k] = discretize_nl_sys(Jac_F, Gamma_c, W_c, mp.dt, taylor_order, false, true);
    % Propagate covariance and state estimates
    P_kf = Phi_k * P_kf * Phi_k' + W_k;
    x_kf = nonlin_transition_model(x_kf);
    % Form Kalman Gain, update estimate and covariance
    K = (H * P_kf * H' + V) \ (H * P_kf)';
    zbar = H * x_kf;
    r = zs(k+1) - zbar;
    x_kf = x_kf + K * r;
    P_kf = (I2 - K * H) * P_kf * (I2 - K * H)' + K * V * K';
    % Store estimates
    xs_kf = [xs_kf; x_kf'];
    Ps_kf(k, :, :) = P_kf;
end

% Plot Simulation results 
plot_simulation_history([], {xs,zs,ws,vs}, {xs_kf, Ps_kf});

%% Cauchy
scale_g2c = 1.0 / 1.3898; % scale factor to fit the cauchy to the gaussian
beta = sqrt(mp.w_PSD / mp.dt) * scale_g2c / 50;
gamma = sqrt(V(1, 1)) * scale_g2c;
x0_ce = x0_kf;
A0 = eye(3);
p0 = sqrt(diag(P0_kf)) * scale_g2c;
b0 = zeros(3, 1);
steps = 5;
num_controls = 0;
print_debug = false;

% Uncomment for sanity check and test using MCauchyEstimator

% cauchyEst = MCauchyEstimator("nonlin", steps, print_debug);

% cauchyEst.initialize_nonlin(x0_ce, A0, p0, b0, beta, gamma, 'dynamics_update', 'nonlinear_msmt_model', 'msmt_model_jacobian', num_controls, mp.dt)

% cauchyEst.step(zs(1));
% cauchyEst.step(zs(2));
% cauchyEst.step(zs(3));
% cauchyEst.step(zs(4));
% cauchyEst.step(zs(5));
% cauchyEst.shutdown();

% Sliding window
swm_print_debug = false; 
win_print_debug = false;
% num_windows = 8;
num_windows = 4;
% 
cauchyEst = MSlidingWindowManager("nonlin", num_windows, swm_print_debug, win_print_debug);
cauchyEst.initialize_nonlin(x0_ce, A0, p0, b0, beta, gamma, 'dynamics_update', 'nonlinear_msmt_model', 'msmt_model_jacobian', num_controls, mp.dt);
% charACTERIZE the rate
for k = 1:length(zs)
    zk = zs(k);
    [xhat, Phat, wavg_xhat, wavg_Phat] = cauchyEst.step(zk, []);
end
cauchyEst.shutdown()

plot_simulation_history(cauchyEst.moment_info, {xs,zs,ws,vs}, {xs_kf, Ps_kf} )



%%


% function [xs, xs_kf, xs_cf] = test_nl_motor_pendulum()
%     
% 
%     % simulation parameters
%     A_c = [-(B/J + K_m^2/(J*R))]; % single dimensional system with omega (no theta yet)
%     B_c = [K_m*V_s/(J*R)];
%     C_c = [1];
% 
% end

figure;
ax(1) = subplot(2,1,1);
plot(Ts, xs_kf(:,1)); hold on;
plot(Ts, cauchyEst.moment_info.x(:,1)); hold on;
plot(Ts, xs(:,1),'k--'); hold on;
legend('Kalman','Cauchy','Sim','Orientation','horizontal');
title('Position','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Position [rad]','Interpreter','latex');
grid on; % grid minor;

ax(2) = subplot(2,1,2);
plot(Ts, xs_kf(:,2)); hold on;
plot(Ts, cauchyEst.moment_info.x(:,2)); hold on;
plot(Ts, xs(:,2),'k--'); hold on;
legend('Kalman','Cauchy','Sim');
title('Velocity','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Velocity [rad/s]','Interpreter','latex');
grid on; % grid minor;

linkaxes(ax,'x');
