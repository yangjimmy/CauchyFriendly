% clc; clear; close all;

addpath("simulink");

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

feedback_control = false;
% feedback_control = true;


%% Simulink setup
% Initial state
pendulum_initial_pos = pi/2;
pendulum_initial_vel = 0;

x0 = [0; 0];


% K = -[0.1151    0.0112];
% K = zeros(1,2);
load('lqr_K.mat')
K
K = -K;


% K = -[0.0289, 0.0048];

paramBus = Simulink.Bus.createObject(mp);

simres = sim('simu_model3_feedback.slx');


%%
t   = simres.simout.states.Time;
pos = simres.simout.states.Data(:,1);
vel = simres.simout.states.Data(:,2);
pos_m = simres.simout.meas.Data;
u = simres.simout.u.Data;

% vel_m = simres.simout.vel_m.Data;

data.t = t;
data.pos = pos;
data.vel = vel;
data.pos_m = pos_m;
data.u = u;
% data.vel_m = vel_m;


figure;
stairs(t, pos);
grid on;
figure;
stairs(t, vel);
grid on;
figure;
stairs(t, u);
grid on;


% if enableNoise
%     save('.\data\pendulum_wall_simu_noise','data');
% else
%     save('.\data\pendulum_wall_simu','data');
% end

if feedback_control
    save('.\data\pendulum_wall_simu_fb','data');
else
    save('.\data\pendulum_wall_simu','data');
end

