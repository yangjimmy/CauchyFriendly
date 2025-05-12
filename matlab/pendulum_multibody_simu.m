

addpath("Model3");

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

paramBus = Simulink.Bus.createObject(mp);

simres = sim('simu_model3.slx');


%%
t   = simres.simout.pos.Time;
pos = simres.simout.pos.Data;
vel = simres.simout.vel.Data;
pos_m = simres.simout.pos_m.Data;
% vel_m = simres.simout.vel_m.Data;

data.t = t;
data.pos = pos;
data.vel = vel;
data.pos_m = pos_m;
% data.vel_m = vel_m;


figure;
plot(t, rad2deg(pos));
figure; plot(t, vel);

if enableNoise
    save('.\data\pendulum_wall_simu_noise','data');
else
    save('.\data\pendulum_wall_simu','data');
end

