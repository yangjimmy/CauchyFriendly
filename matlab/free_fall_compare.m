clc; clear; close all;

dataExp = load('.\data\pendulum_0415_04_pos.mat');
load('.\data\pendulum_free_simu.mat');
dataSimulink = data;
load('.\data\pendulum_simulation_free.mat');
dataSim = data;



%%
figure;
tiledlayout(2,1,'TileSpacing','tight');
nexttile;
plot(dataSim.t, dataSim.pos,'LineWidth',1.3); hold on;
plot(dataSimulink.t, dataSimulink.pos,'LineWidth',1.3); hold on;
plot(dataExp.t, dataExp.pos,'--','LineWidth',1.3); hold on;
% xlabel('Time [s]','Interpreter','latex');
ylabel('Pos [rad]','Interpreter','latex');
grid on;
legend('Simulation','Simulink','Experiment','Interpreter','latex', ...
    'Fontsize',12,'Location','northeast','orientation','horizontal');

nexttile;
plot(dataSim.t, dataSim.vel,'LineWidth',1.3); hold on
plot(dataSimulink.t, dataSimulink.vel,'LineWidth',1.3); hold on;
plot(dataExp.t, dataExp.vel,'--','LineWidth',1.3); hold on;
xlabel('Time [s]','Interpreter','latex');
ylabel('Vel [rad/s]','Interpreter','latex');
grid on;

sgtitle('Simulation, Simulink, Experiment', ...
    'Interpreter','latex');

exportgraphics(gcf,'.\fig\free_fall_comparison.png','resolution',600);

