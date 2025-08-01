clc; clear; close all;

addpath("simulink");
addpath("filter_callbacks")
addpath("mp_callbacks");
addpath("matlab_pure");
% Revolute joint state
% smiData.RevoluteJoint(1).Rz.Pos

%% Call pendulum parameter
% generate a `mp` file logging all pendulum parameters
if ~exist('mp','var')
    pendulum_DataFile
end

%% Simulation configuration
simTime = 4; % s
samplingTime = mp.dt;
samplingRate = 1 / samplingTime;
enableNoise = true;
% enableNoise = false;


%% Simulink setup
% Initial state
pendulum_initial_pos = pi/2;
pendulum_initial_vel = 0;
theta_vec0 = [pendulum_initial_pos; pendulum_initial_vel];

% Kalman filter initial values
P0_kf = eye(2) * 0.003;             % Initial variance
x0_kf = mvnrnd(theta_vec0, P0_kf).';  % Initial mean

paramBus = Simulink.Bus.createObject(mp);

%%
simres = sim('simu_model3_Kalman_c.slx');


%%
t   = simres.simout.pos.Time;
pos = simres.simout.pos.Data;
vel = simres.simout.vel.Data;
pos_m = simres.simout.pos_m.Data;

data.t = t;
data.pos = pos;
data.vel = vel;
data.pos_m = pos_m;


figure;
plot(t, rad2deg(pos));
figure; plot(t, vel);

% if enableNoise
%     save('.\data\pendulum_wall_simu_noise','data');
% else
%     save('.\data\pendulum_wall_simu','data');
% end

