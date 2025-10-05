clear; clc; close all;

addpath("matlab_pure");
addpath("mp_callbacks");
addpath("mex_files");
addpath("filter_callbacks");
addpath(pwd);

k_std_color = 'b--';
k_err_color = 'r';
c_std_color = 'g--';
c_err_color = 'k';

%% Path of the data
% dataPath = '.\data\pendulum_wall_0603_01.mat';
% dataPath = '.\data\pendulum_wall_0424_05_pos.mat';
% dataPath = '.\data\pendulum_0415_04_pos.mat';

% data = load(dataPath);

% dataPath = '..\..\..\Analysis\data_log_cauchy_250Hz_res100_4.csv';
dataPath = '..\..\..\Analysis\data_log_cauchy_250Hz_res400_fb_4.csv';
% dataPath = '..\..\..\Analysis\data_log_cauchy_250Hz_res400_fb_d_4.csv';
data = readmatrix(dataPath);

%% Call pendulum parameter
% generate a `mp` file logging all pendulum parameters
if ~exist('mp','var')
    pendulum_DataFile
end
num_state = 2;

Ts = 0.004;

%% Simulation setup
theta_vec0 = [pi/2; 0];     % initial angle of 90 degrees at 0 radians/sec

zs = data(:,1);
xs = [data(:,2), data(:,3)];
u = data(:,8);
% u = zeros(size(data));

propagations = length(zs) -1;
Ts = (0:(propagations)) * 0.004;

%% Kalman Filter       % 2x2 identity matrix
taylor_order = 2;       % order of taylor expansion for transition matrix approx.

% Setting up and running the EKF
% The gaussian_filters module has a "run_ekf" function baked in, but we'll just show the whole thing here
P0_kf = eye(2) * sqrt(mp.v_PSD);             % Initial variance
% x0_kf = mvnrnd(theta_vec0, P0_kf);  % Initial mean
x0_kf = theta_vec0.';
[xs_kf, Ps_kf] = propagate_kf_nl(x0_kf,P0_kf,zs,u,propagations,taylor_order);

new_data = [data, xs_kf, Ps_kf(:,1,1), Ps_kf(:,2,2)];
% savePath = '..\..\..\Analysis\data_log_cauchy_kalman_250Hz_res050_4.csv';
savePath = '..\..\..\Analysis\data_log_cauchy_kalman_250Hz_res400_fb_4.csv';
% savePath = '..\..\..\Analysis\data_log_cauchy_kalman_250Hz_res400_fb_d_4.csv';
writematrix(new_data, savePath)

