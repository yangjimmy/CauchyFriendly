clc; clear; close all;

dataExp = load('.\data\pendulum_wall_0424_05_pos.mat');
load('.\data\pendulum_wall_simu.mat');
dataSim = data;

%%
figure;
tiledlayout(2,1,'TileSpacing','tight');
nexttile;
plot(dataExp.t, dataExp.pos); hold on;
plot(dataSim.t, dataSim.pos); hold on;
grid on;
nexttile;
plot(dataExp.t, dataExp.vel); hold on;
plot(dataSim.t, dataSim.vel); hold on;
grid on;

legend('Experiment','Simulation','Interpreter','latex','Fontsize',12);

