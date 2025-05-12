clc; clear; close all;

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
% enableNoise = true;
enableNoise = false;


%% Simulink setup
% Initial state
pendulum_initial_pos = pi/2;
pendulum_initial_vel = 0;

paramBus = Simulink.Bus.createObject(mp);

simres = sim('simu_model_3state.slx');


%%
t   = simres.simout.pos.Time;
pos = simres.simout.pos.Data;
vel = simres.simout.vel.Data;
pos_m = simres.simout.pos_m.Data;
cur = simres.simout.cur.Data;
% vel_m = simres.simout.vel_m.Data;

data.t = t;
data.pos = pos;
data.vel = vel;
data.pos_m = pos_m;
data.cur = cur;
% data.vel_m = vel_m;


figure;
nexttile;
plot(t, rad2deg(pos));
nexttile;
plot(t, vel);
nexttile;
plot(t, cur);
% if enableNoise
%     save('.\data\pendulum_wall_simu_noise','data');
% else
%     save('.\data\pendulum_wall_simu','data');
% end

% save('.\data\pendulum_free_simu','data');



% %% Write video
% v = VideoWriter('my_plot_video.avi', 'Motion JPEG AVI');  % Create video object
% v.FrameRate = 60;                      % Set frame rate
% v.Quality = 100;
% open(v);                               % Open the video file for writing
% 
% figure('Position',[200,50, 700, 900]);
% for i = 1:16:length(t)
%     subplot(3,1,1);
%     plot(t(1:i), pos(1:i));
%     ylim([min(pos), max(pos)]); xlim([0 t(end)]);
%     grid on;
%     ylabel('Pos [rad]','Interpreter','latex');
%     subplot(3,1,2);
%     plot(t(1:i), vel(1:i));
%     ylim([min(vel), max(vel)]); xlim([0 t(end)]);
%     grid on;
%     ylabel('Vel [rad/s]','Interpreter','latex');
%     subplot(3,1,3);
%     plot(t(1:i), cur(1:i));
%     ylim([min(cur), max(cur)]); xlim([0 t(end)]);
%     grid on;
%     ylabel('Cur [Amp]','Interpreter','latex');
%     xlabel('Time [s]','Interpreter','latex');
% 
%     % title(sprintf('Time = %.1f seconds', t(i)));
%     % xlabel('Time');
%     % ylabel('Amplitude');
%     % grid on;
% 
%     frame = getframe(gcf);             % Capture current frame
%     writeVideo(v, frame);              % Write frame to video
% end

close(v);

