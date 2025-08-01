clear; clc; close all;

% rmpath("nl_tut_callbacks")
addpath("matlab_pure");
addpath("mp_callbacks");
addpath("mex_files");
addpath(pwd);

%% Call pendulum parameter
% generate a `mp` file logging all pendulum parameters
if ~exist('mp','var')
    pendulum_DataFile
end

%% Bode Transfer Function
% s = tf('s');
% basic_tf = (mp.K_m*mp.V_s/(mp.J*mp.R))/(s^2+s*(mp.B/mp.J+mp.K_m^2/(mp.J*mp.R)));
% bode(basic_tf);

%% Simulation setup
theta_vec0 = [pi/2; 0];     % initial angle of 90 degrees at 0 radians/sec
sim_time = 5;               % s
propagations = sim_time / mp.dt;
data_length = propagations + 1;
num_state = 2;

% system characteristics
V = mp.v_PSD;       % noise variance
W = mp.w_PSD;

% model
H = [1.0, 0.0];             % meausrement model
Gamma_c = [0.0; 1.0];       % Continuous time Gamma (\Gamma_c)


%% Generate a trajectory
% theta_k = theta_vec0;
% --- Generate trajectory with nonlinear transition model
thetas = zeros(num_state,propagations+1);
thetas(:,1) = theta_vec0;
for k = 1:propagations
    theta_k = nonlin_transition_model(thetas(:,k));
    thetas(:,k+1) = theta_k;
end
thetas = thetas.';

% --- Generate trajectory with ode45 solver (instead of runge_kutta4)
% tspan = 0:mp.dt:sim_time;
% sol = ode45(@trajectory_propagate,[0, sim_time],theta_vec0);
% thetas = deval(sol,tspan).';

Ts = ((0:propagations) * mp.dt)';
figure;
sgtitle('Pendulum Trajectory (angle: top), (angular rate: bottom)');
subplot(2, 1, 1);
plot(Ts, thetas(:, 1));
subplot(2, 1, 2);
plot(Ts, thetas(:, 2));

%% Generate trajectory with noise in simulation
% Initialize vectors
xs = zeros(num_state,data_length);
zs = zeros(data_length,1);

% generate the process and sensor noise
ws = sqrt(W) * randn(data_length,1) * mp.dt;
vs = unifrnd(-mp.Enc_n,mp.Enc_n,data_length,1);

% initial condition setup
xs(:,1) = theta_vec0;
zs(1) = H * xs(:,1) +vs(1);

for k = 1:propagations
    wk = sqrt(W) * randn() * mp.dt;
    xk = xs(:,k);
    xk = nonlin_transition_model(xk);
    % [~,xk] = ode45(@trajectory_propagate,[0, mp.dt],xk);
    % xk = xk(end,:).';
    % Dynamic equation: dx = Phi xdt + Gamma dw
    xk = xk + Gamma_c * ws(k+1);
    xs(:,k+1) = xk;
    zs(k+1) = H * xk + vs(k+1);
end
xs = xs.';          % transpose xs so it's stacked row vectors

plot_simulation_history([], {xs,zs,ws,vs}, [])

%% Kalman Filter
I2 = eye(2);            % 2x2 identity matrix
taylor_order = 2;       % order of taylor expansion for transition matrix approx.

% Setting up and running the EKF
% The gaussian_filters module has a "run_ekf" function baked in, but we'll just show the whole thing here
P0_kf = eye(2) * 0.003;             % Initial variance
x0_kf = mvnrnd(theta_vec0, P0_kf);  % Initial mean

% initialize
xs_kf = zeros(num_state,data_length);
Ps_kf = zeros(data_length, 2, 2);

% initial condition
xs_kf(:,1) = x0_kf;
Ps_kf(1, :, :) = P0_kf;

% propagation vectors
x_kf = x0_kf';
P_kf = P0_kf;

for k = 1:propagations
    % characterize nonlinear dynamics
    Jac_F = jacobian_mp_ode(x_kf);
    [Phi_k, W_k] = discretize_nl_sys(Jac_F, Gamma_c, W, mp.dt, taylor_order, false, true);

    % Propagate covariance and state estimates
    P_kf = Phi_k * P_kf * Phi_k' + W_k;
    x_kf = nonlin_transition_model(x_kf);
    % [~,x_kf] = ode45(@trajectory_propagate,[0, mp.dt],x_kf);
    % x_kf = x_kf(end,:).';

    % Form Kalman Gain, update estimate and covariance
    K = (H * P_kf * H' + V) \ (H * P_kf)';
    zbar = H * x_kf;            % estimated measurement
    r = zs(k+1) - zbar;         % residue
    x_kf = x_kf + K * r;        % updated state
    P_kf = (I2 - K * H) * P_kf * (I2 - K * H)' + K * V * K';    % covariance update

    % Store estimates
    xs_kf(:,k+1) = x_kf;
    Ps_kf(k, :, :) = P_kf;
end
xs_kf = xs_kf.';

% Plot Simulation results 
plot_simulation_history([], {xs,zs,ws,vs}, {xs_kf, Ps_kf});

%% Cauchy
scale_g2c = 1.0 / 1.3898;       % scale factor to fit the cauchy to the gaussian
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
% num_windows = 8;
num_windows = 6;
tic;
% 
cauchyEst = MSlidingWindowManager("nonlin", num_windows, swm_print_debug, win_print_debug);
cauchyEst.initialize_nonlin(x0_ce, A0, p0, b0, beta, gamma, 'dynamics_update', 'nonlinear_msmt_model', 'msmt_model_jacobian', num_controls, mp.dt);
% charACTERIZE the rate
for k = 1:length(zs)
    zk = zs(k);
    [xhat, Phat, wavg_xhat, wavg_Phat] = cauchyEst.step(zk, []);
end
cauchyEst.shutdown()
toc;
plot_simulation_history(cauchyEst.moment_info, {xs,zs,ws,vs}, {xs_kf, Ps_kf} )

%% Plot the result
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

%%
data.pos = xs(:,1);
data.vel = xs(:,2);

clear ax;
figure('Position',[200,200,1000,500]);
tiledlayout(1,2,"TileSpacing",'compact');
% ax(1) = subplot(2,1,1);
ax(1) = nexttile;
plot(Ts, xs_kf(:,1)); hold on;
plot(Ts, cauchyEst.moment_info.x(:,1),'linewidth',1.5); hold on;
plot(Ts, data.pos,'k--'); hold on;
legend('Kalman','Cauchy','Simulation','Interpreter','latex','FontSize',12);
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
