clc; clear; close all;

% load('.\data\pendulum_wall_0414_01.mat');
% load('.\data\pendulum_wall_0414_03.mat');
% load('.\data\pendulum_0415_04.mat');
% load('.\data\pendulum_wall_0424_02.mat');
load('.\data\pendulum_wall_0424_05.mat');


Ts = 1e-3;
Fs = 1/Ts;
factor = 5;
swingTime = 3;

t = simData.pos.Time;
pos = simData.pos.Data;
vel = simData.vel.Data;

figure;
ax(1) = nexttile;
plot(t, pos);
grid minor; axis tight;
ax(2) = nexttile;
plot(t, vel);
grid minor; axis tight;
linkaxes(ax,'x');

% trimmer = t>=11.965;
% trimmer = t>=11.486;
% trimmer = t>=9.089;
% trimmer = trimmer(1:5 * Fs+1);
% trimmer = 1:length(t);

% trimmer = t>16.778;
trimmer = t > 8.771;

% t = t(trimmer);
t = t - t(1);
pos = pos(trimmer) / factor;
vel = vel(trimmer) / factor;

t = t(1:swingTime*Fs+1);
pos = pos(1:swingTime*Fs+1);
vel = vel(1:swingTime*Fs+1);

figure;
ax(1) = nexttile;
plot(t, pos);
grid minor; axis tight;
ax(2) = nexttile;
plot(t, vel);
grid minor; axis tight;
linkaxes(ax,'x');

% save('.\data\pendulum_wall_0414_01_pos.mat','t','pos','vel');
% save('.\data\pendulum_wall_0414_03_pos.mat','t','pos','vel');
% save('.\data\pendulum_0415_04_pos.mat','t','pos','vel');
save('.\data\pendulum_wall_0424_05_pos.mat','t','pos','vel');
