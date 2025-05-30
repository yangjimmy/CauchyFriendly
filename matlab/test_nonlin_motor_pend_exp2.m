clear; clc; close all;

% rmpath("nl_tut_callbacks")
addpath("matlab_pure");
addpath("mp_callbacks");
addpath("mex_files");
addpath("filter_callbacks")
addpath(pwd);

%% Path of the data
% dataPath = '.\data\pendulum_wall_0414_01_pos.mat';
% dataPath = '.\data\pendulum_wall_0414_03_pos.mat';
% dataPath = '.\data\pendulum_0415_04_pos.mat';
dataPath = '.\data\pendulum_wall_0424_05_pos.mat';

data = load(dataPath);

%% Call pendulum parameter
% generate a `mp` file logging all pendulum parameters
if ~exist('mp','var')
    pendulum_DataFile
end

%% Simulation setup
theta_vec0 = [pi/2; 0];     % initial angle of 90 degrees at 0 radians/sec
% sim_time = 4;               % s
propagations = length(data.t) - 1;

% system characteristics
V = mp.v_PSD;       % noise variance
W = mp.w_PSD;

% model
mp.H = [1.0, 0.0];             % meausrement model
mp.Gamma_c = [0.0; 1.0];       % Continuous time Gamma (\Gamma_c), multiplies process noise w

%% Take the measurement from data
Ts = data.t;
zs = data.pos;
xs = [data.pos, data.vel];

%% Kalman Filter       % 2x2 identity matrix
taylor_order = 2;       % order of taylor expansion for transition matrix approx.

% Setting up and running the EKF
% The gaussian_filters module has a "run_ekf" function baked in, but we'll just show the whole thing here
P0_kf = eye(2) * 0.003;             % Initial variance
x0_kf = mvnrnd(theta_vec0, P0_kf);  % Initial mean

[xs_kf, Ps_kf] = propagate_kf_nl(x0_kf,P0_kf,zs,propagations,taylor_order);

% Plot Simulation results 
% plot_simulation_history([], {xs,zs,ws,vs}, {xs_kf, Ps_kf});

%% Cauchy
scale_g2c = 1.0 / 1.3898;
cauchyEst = propagate_cf_nl(x0_kf,P0_kf,zs,scale_g2c,propagations);
% plot_simulation_history(cauchyEst.moment_info, {xs,zs,ws,vs}, {xs_kf, Ps_kf} )

%%
figure;
for idx = 1:num_state
    ax(idx) = subplot(num_state,1,idx);
    plot(Ts(2:end), sqrt(cauchyEst.moment_info.P(:,idx,idx)),'r'); hold on;
    plot(Ts(2:end),-sqrt(cauchyEst.moment_info.P(:,idx,idx)),'r'); hold on;
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
c_e = cauchyEst.moment_info.x(:,idx) - data.pos(2:end);
kf_std = sqrt(Ps_kf(:,idx,idx));
c_std = sqrt(cauchyEst.moment_info.P(:,idx,idx));

plot(Ts, kf_e); hold on;
plot(Ts(2:end), c_e); hold on;
plot(Ts(2:end),  c_std,'r'); hold on;
plot(Ts(2:end), -c_std,'r'); hold on;
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
c_e = cauchyEst.moment_info.x(:,idx) - data.vel(2:end);
kf_std = sqrt(Ps_kf(:,idx,idx));
c_std = sqrt(cauchyEst.moment_info.P(:,idx,idx));

plot(Ts, kf_e); hold on;
plot(Ts(2:end), c_e); hold on;
plot(Ts(2:end),  c_std,'r'); hold on;
plot(Ts(2:end), -c_std,'r'); hold on;
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
plot(Ts(2:end), cauchyEst.moment_info.x(:,1),'linewidth',1.5); hold on;
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
plot(Ts(2:end), cauchyEst.moment_info.x(:,2),'linewidth',1.5); hold on;
plot(Ts, data.vel,'k--'); hold on;
% title('Velocity','Interpreter','latex');
xlabel('Time [s]','Interpreter','latex','FontSize',14);
ylabel('Velocity [rad/s]','Interpreter','latex','FontSize',14);
grid on; axis tight;
xlim([0,1.5]);

linkaxes(ax,'x');

% exportgraphics(gcf,'.\fig\exp_states.png','Resolution',600);
