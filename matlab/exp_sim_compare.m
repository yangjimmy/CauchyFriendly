clc; clear; close all;

dataExp = load('.\data\pendulum_wall_0424_05_pos.mat');
load('.\data\pendulum_wall_simu_noise.mat');
dataSim = data;

%%
figure;
tiledlayout(2,1,'TileSpacing','tight');
nexttile;
plot(dataSim.t, dataSim.pos,'LineWidth',1.3); hold on;
plot(dataExp.t, dataExp.pos,'LineWidth',1.3); hold on;
ylabel('Pos [rad]','Interpreter','latex');
grid on;
legend('Simulink','Experiment','Interpreter','latex', ...
    'Fontsize',12,'Location','northeast');

nexttile;
plot(dataSim.t, dataSim.vel,'LineWidth',1.3); hold on;
plot(dataExp.t, dataExp.vel,'LineWidth',1.3); hold on;
xlabel('Time [s]','Interpreter','latex');
ylabel('Vel [rad/s]','Interpreter','latex');
grid on;

sgtitle('Simulation vs Experimental Result', ...
    'Interpreter','latex');

exportgraphics(gcf,'.\fig\exp_sim_comparison.png','resolution',600);

