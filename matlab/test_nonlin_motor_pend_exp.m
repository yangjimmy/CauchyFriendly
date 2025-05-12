clear; clc; close all;

% global mp
rmpath("nl_tut_callbacks")
addpath("matlab_pure");
addpath("mp_callbacks");
addpath("mex_files");
addpath(pwd);

%% Path of the data
dataPath = '.\data\pendulum_wall_0414_01_pos.mat';
dataPath = '.\data\pendulum_wall_0414_03_pos.mat';
% dataPath = '.\data\pendulum_0415_04_pos.mat';
dataPath = '.\data\pendulum_wall_0424_05_pos.mat';

data = load(dataPath);


%%
%% Call pendulum parameter
% generate a `mp` file logging all pendulum parameters
if ~exist('mp','var')
    pendulum_DataFile
end

%% Bode Transfer Function
s = tf('s');
basic_tf = (mp.K_m*mp.V_s/(mp.J*mp.R))/(s^2+s*(mp.B/mp.J+mp.K_m^2/(mp.J*mp.R)));
bode(basic_tf);

% TODO: pendulum colliding with wall

%% Generate a trajectory
theta_vec0 = [pi/2; 0]; % initial angle of 45 degrees at 0 radians/sec
theta_k = theta_vec0;
thetas = theta_k';
% propagations = 160;
propagations = length(data.t) - 1;

% for k = 1:propagations
%     theta_k = nonlin_transition_model(theta_k);
%     thetas = [thetas; theta_k'];
% end
% Ts = ((0:propagations) * mp.dt)';
% figure;
% sgtitle('Pendulum Trajectory (angle: top), (angular rate: bottom)');
% subplot(2, 1, 1);
% plot(Ts, thetas(:, 1));
% subplot(2, 1, 2);
% plot(Ts, thetas(:, 2));

% plot makes sense since back EMF prevents the motor from rotating 

%% Generate trajectory with noise
% Creating the dynamic simulation
% V = 0.009311^2; % best fit Gaussian
% V = mp.Enc_n^2/3 * sqrt(mp.dt);
V = mp.v_PSD;

H = [1.0, 0.0]; % meausrement model
xk = theta_vec0;
xs = xk'; % State vector history
ws = [];   % Process noise history
vs = unifrnd(-pi/200,pi/200); % Measurement noise history; sample from uniform distribution
% zs = H * xk + vs(1); % Measurement history
% propagations = 160;
% for k = 1:propagations
%     wk = mp.dt * sqrt(mp.w_PSD) * randn();
%     xk(2) = xk(2) + wk; % Gamma = [0; 1];
%     xk = nonlin_transition_model(xk);
%     xs = [xs; xk'];
%     ws = [ws; wk];
%     vk = unifrnd(-pi/200,pi/200); % sample from uniform distribution
%     zk = H * xk + vk;
%     vs = [vs; vk];
%     zs = [zs; zk];
% end
% plot_simulation_history([], {xs,zs,ws,vs}, [])
% 
% ws = mp.dt * sqrt(mp.w_PSD) * randn(size(data1.t));
% vs = unifrnd(-pi/200,pi/200,size(data1.t));
% ws = zeros(size(data1.t));
% vs = zeros(size(data1.t));

Ts = data.t;
zs = data.pos;
xs = [data.pos, data.vel];

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
% plot_simulation_history([], {xs,zs,ws,vs}, {xs_kf, Ps_kf});

%% Cauchy
scale_g2c = 1.0 / 1.3898; % scale factor to fit the cauchy to the gaussian
beta = sqrt(mp.w_PSD / mp.dt) * scale_g2c;
gamma = sqrt(V(1, 1)) * scale_g2c;
x0_ce = x0_kf;
A0 = eye(2);
p0 = sqrt(diag(P0_kf)) * scale_g2c;
b0 = zeros(2, 1);
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

% plot_simulation_history(cauchyEst.moment_info, {xs,zs,ws,vs}, {xs_kf, Ps_kf} )



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







%%
% plot_simulation_history(cauchyEst.moment_info, {}, {xs_kf, Ps_kf} )


figure;
for idx = 1:2
    ax(idx) = subplot(2,1,idx);
    plot(Ts, sqrt(cauchyEst.moment_info.P(:,idx,idx)),'r'); hold on;
    plot(Ts,-sqrt(cauchyEst.moment_info.P(:,idx,idx)),'r'); hold on;
    plot(Ts, sqrt(Ps_kf(:,idx,idx)),'m'); hold on;
    plot(Ts,-sqrt(Ps_kf(:,idx,idx)),'m'); hold on;
    legend('Cauchy 1-Sig bound','','Kalman 1-Sig bound','','interpreter','latex');
    grid on;
end

linkaxes(ax,'x');

%%
clear ax;
figure('Position',[200,200,1000,500]);
tiledlayout(1,2,'TileSpacing','compact')
% ax(1) = subplot(2,1,1);
ax(1) = nexttile;
idx = 1;
kf_e = xs_kf(:,idx) - data.pos;
c_e = cauchyEst.moment_info.x(:,idx) - data.pos;
kf_std = sqrt(Ps_kf(:,idx,idx));
c_std = sqrt(cauchyEst.moment_info.P(:,idx,idx));

plot(Ts, kf_e); hold on;
plot(Ts, c_e); hold on;
plot(Ts,  c_std,'r'); hold on;
plot(Ts, -c_std,'r'); hold on;
plot(Ts,  kf_std,'m'); hold on;
plot(Ts, -kf_std,'m'); hold on;
% title('Simulated error','Interpreter','latex');
grid on; 
xlim([0.18,0.4]);
% xlim([.18,.25]);
% ylim([-.06,.02]);

% ylim([-max(abs(kf_e)),max(abs(kf_e))]);
xlabel('Time [s]','Interpreter','latex','FontSize',14);
ylabel('Position [rad]','Interpreter','latex','FontSize',14);
legend('Kalman error','Cauchy error','Cauchy 1-Sig bound','','Kalman 1-Sig bound','', ...
    'Interpreter','latex','Location','southeast','FontSize',12);

% ax(2) = subplot(2,1,2);
ax(2) = nexttile;
idx = 2;
kf_e = xs_kf(:,idx) - data.vel;
c_e = cauchyEst.moment_info.x(:,idx) - data.vel;
kf_std = sqrt(Ps_kf(:,idx,idx));
c_std = sqrt(cauchyEst.moment_info.P(:,idx,idx));

plot(Ts, kf_e); hold on;
plot(Ts, c_e); hold on;
plot(Ts,  c_std,'r'); hold on;
plot(Ts, -c_std,'r'); hold on;
plot(Ts,  kf_std,'m'); hold on;
plot(Ts, -kf_std,'m'); hold on;
% title('Simulated error','Interpreter','latex');
grid on; 
xlim([.18,.4]);
% xlim([.18,.25]);
% ylim([-11,2]);

ylabel('Velocity [rad/s]','Interpreter','latex','FontSize',14);
xlabel('Time [s]','Interpreter','latex','FontSize',14);


sgtitle('Error and 1-sigma bound - Experiment','Interpreter','latex');

linkaxes(ax,'x');

% exportgraphics(gcf,'.\fig\exp_error_sigma_1.png','Resolution',600);



%%
clear ax;
figure('Position',[200,200,1000,500]);
tiledlayout(1,2,"TileSpacing",'compact');
% ax(1) = subplot(2,1,1);
ax(1) = nexttile;
plot(Ts, xs_kf(:,1)); hold on;
plot(Ts, cauchyEst.moment_info.x(:,1),'linewidth',1.5); hold on;
plot(Ts, data.pos,'k--'); hold on;
legend('Kalman','Cauchy','Exp','Interpreter','latex','FontSize',12);
% title('Position','Interpreter','latex');
% xlabel('Time [s]','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex','FontSize',14);
ylabel('Position [rad]','Interpreter','latex','FontSize',14);
grid on; axis tight;
xlim([0,1.5]);

% ax(2) = subplot(2,1,2);
ax(2) = nexttile;
plot(Ts, xs_kf(:,2)); hold on;
plot(Ts, cauchyEst.moment_info.x(:,2),'linewidth',1.5); hold on;
plot(Ts, data.vel,'k--'); hold on;
% title('Velocity','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex','FontSize',14);
ylabel('Velocity [rad/s]','Interpreter','latex','FontSize',14);
grid on; axis tight;
xlim([0,1.5]);

linkaxes(ax,'x');

% exportgraphics(gcf,'.\fig\exp_states.png','Resolution',600);
