clc; clear; close all;

load('.\data\pendulum_free_simu_3.mat');
dataSim = data;
load('.\data\pendulum_0415_04_pos.mat');
data.pos = pos;
data.t = t;
data.vel = vel;
dataExp = data;


figure('Position',[200,200,1000,500]);
tiledlayout(1,2,"TileSpacing",'compact');
ax(1) = nexttile;
plot(dataSim.t, dataSim.pos); hold on;
plot(dataExp.t, dataExp.pos); hold on;
legend('Sim',"Exp",'Interpreter','latex');
xlabel('Time [s]','Interpreter','latex');
ylabel('Position [rad]','Interpreter','latex');

ax(2) = nexttile;
plot(dataSim.t, dataSim.vel); hold on;
plot(dataExp.t, dataExp.vel); hold on;
xlabel('Time [s]','Interpreter','latex');
ylabel('Velocity [rad/s]','Interpreter','latex');


exportgraphics(gcf,'.\fig\exp_3state_compare.png','Resolution',600);
