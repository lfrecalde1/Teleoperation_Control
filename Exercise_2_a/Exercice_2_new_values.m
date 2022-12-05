% Optimization Teleoperation system

% Clean varibales
clc, clear all, close all;
% Gains slave and master

load("Parameters.mat");
% General vector of the variables Gains No delay
%X = [3.94014191427969;chi(1);0.896799897417479;chi(2)];
X = [2.41218473361655;chi(1);1.96805547864400;chi(2)];
% System delay
h1 = 0.5;
h2 = 0.5;

% Final time definition
t_final = 40;

% Phisycal parameters
% System parameters L1 slave
b1_s = 0;
m1_s = 2;
l1_s = 0.45;
Iz1_s= 0.0;

% System parameters L2 slave
b2_s = 0;
m2_s = 1.2;
l2_s = 0.3;
Iz2_s= 0.0;

L1_s = [b1_s , m1_s, l1_s, Iz1_s];
L2_s = [b2_s , m2_s, l2_s, Iz2_s];

% System parameters L1 master
b1_m = 0.0;
m1_m = 2;
l1_m = 0.45;
Iz1_m= 0.0;

% System parameters L2 master
b2_m = 0.0;
m2_m = 1.2;
l2_m = 0.3;
Iz2_m= 0.0;


L1_m = [b1_m , m1_m, l1_m, Iz1_m];
L2_m = [b2_m , m2_m, l2_m, Iz2_m];

% inital conditions Slave
qs = [180*pi/180, 170*pi/180;...
           -30*pi/180, -20*pi/180;...
           0, 0;...
           0, 0];
% Initial Conditions Master
qm = [80*pi/180, 90*pi/180;...
             10*pi/180, 20*pi/180;...
           0, 0;...
           0, 0];

initial = 1;
% System simulation
[q_s, q_m, q_s_delay, q_m_delay, x_s, x_m_base, x_s_delay, x_m_base_delay, xp_s, xp_m, xp_s_delay, xp_m_delay, he_m, he_s, RMSE_x_s_new, RMSE_y_s_new, t, u_cartesian_s, u_cartesian_m, x_m_0, e_new] = Tele_system_simu(X, h1, h2, t_final, L1_s, L2_s, L1_m, L2_m, qs(:,initial), qm(:, initial));


% Plot Results
% Plot properties
% define plot properties
lw = 2; % linewidth 1
lwV = 2; % linewidth 2
fontsizeLabel = 9; %11
fontsizeLegend = 9;
fontsizeTicks = 9;
fontsizeTitel = 9;
sizeX = 900; % size figure
sizeY = 300; % size figure

% color propreties
C1 = [246 170 141]/255;
C2 = [51 187 238]/255;
C3 = [0 153 136]/255;
C4 = [238 119 51]/255;
C5 = [204 51 17]/255;
C6 = [238 51 119]/255;
C7 = [187 187 187]/255;
C8 = [80 80 80]/255;
C9 = [140 140 140]/255;
C10 = [0 128 255]/255;
C11 = [234 52 89]/255;
C12 = [39 124 252]/255;
C13 = [40 122 125]/255;
%C14 = [86 215 219]/255;
C14 = [252 94 158]/255;
C15 = [244 171 39]/255;
C16 = [100 121 162]/255;
C17 = [255 0 0]/255;

figure('Position', [10 10 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');

subplot(2,2,1)
plot(t,q_s(1,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,q_s_delay(1,1:length(t)),'--','Color',C12,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[rad]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(Slave)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$q_{1s}(t)$','$q_{1s}(t-h_2)$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;


subplot(2,2,2)
plot(t,q_s(2,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,q_s_delay(2,1:length(t)),'--','Color',C13,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(Slave)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$q_{2s}(t)$','$q_{2s}(t-h_2)$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(2,2,3)
plot(t,q_s(3,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,q_s_delay(3,1:length(t)),'--','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(Slave)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\dot{q}_{1s}(t)$','$\dot{q}_{1s}(t-h_2)$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(2,2,4)
plot(t,q_s(4,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,q_s_delay(4,1:length(t)),'--','Color',C16,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(Slave)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\dot{q}_{2s}(t)$','$\dot{q}_{2s}(t-h_2)$'},'interpreter','latex','fontsize',fontsizeLegend)

figure('Position', [10 10 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');

subplot(2,2,1)
plot(t,q_m(1,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
%plot(uv(1,:),uv(2,:),'-','Color',C11,'LineWidth',lw);
plot(t,q_m_delay(1,1:length(t)),'--','Color',C12,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[rad]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(Master)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$q_{1m}(t)$','$q_{1m}(t-h_1)$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;


subplot(2,2,2)
plot(t,q_m(2,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,q_m_delay(2,1:length(t)),'--','Color',C13,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[rad]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(Master)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$q_{2m}(t)$','$q_{2m}(t-h_1)$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(2,2,3)
plot(t,q_m(3,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,q_m_delay(3,1:length(t)),'--','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(Master)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\dot{q}_{1m}(t)$','$\dot{q}_{1m}(t-h_1)$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(2,2,4)
plot(t,q_m(4,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,q_m_delay(4,1:length(t)),'--','Color',C16,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
%ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(Master)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\dot{q}_{2m}(t)$','$\dot{q}_{2m}(t-h_1)$'},'interpreter','latex','fontsize',fontsizeLegend)


figure('Position', [10 10 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');

subplot(2,2,1)
plot(t,x_s(1,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,x_s_delay(1,1:length(t)),'--','Color',C12,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(Slave)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$x_{s}(t)$','$x_{s}(t-h_2)$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;


subplot(2,2,2)
plot(t,x_s(2,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,x_s_delay(2,1:length(t)),'--','Color',C13,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(Slave)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$y_{s}(t)$','$y_{s}(t-h_2)$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(2,2,3)
plot(t,xp_s(1,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,xp_s_delay(1,1:length(t)),'--','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(Slave)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\dot{x}_{s}(t)$','$\dot{x}_{s}(t-h_2)$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(2,2,4)
plot(t,xp_s(2,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,xp_s_delay(2,1:length(t)),'--','Color',C16,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(Slave)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\dot{y}_{s}(t)$','$\dot{y}_{s}(t-h_2)$'},'interpreter','latex','fontsize',fontsizeLegend)

figure('Position', [10 10 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');

subplot(2,2,1)
plot(t,x_m_base(1,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,x_m_base_delay(1,1:length(t)),'--','Color',C12,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(Slave)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$x_{m}(t)$','$x_{m}(t-h_1)$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;


subplot(2,2,2)
plot(t,x_m_base(2,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,x_m_base_delay(2,1:length(t)),'--','Color',C13,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(Slave)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$y_{m}(t)$','$y_{m}(t-h_1)$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(2,2,3)
plot(t,xp_m(1,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,xp_m_delay(1,1:length(t)),'--','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(Slave)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\dot{x}_{m}(t)$','$\dot{x}_{m}(t-h_1)$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(2,2,4)
plot(t,xp_m(2,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,xp_m_delay(2,1:length(t)),'--','Color',C16,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
%title({'(Slave)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\dot{y}_{m}(t)$','$\dot{y}_{m}(t-h_1)$'},'interpreter','latex','fontsize',fontsizeLegend)


figure('Position', [10 10 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');

subplot(2,2,1)
plot(t,x_s_delay(1,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,x_m_base_delay(1,1:length(t)),'--','Color',C12,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(Position x)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$x_s(t-h_2)$','$x_m(t-h_1)$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;


subplot(2,2,2)
plot(t,x_s_delay(2,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,x_m_base_delay(2,1:length(t)),'--','Color',C13,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(Position y)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$y_s(t-h_2)$','$y_m(t-h_1)$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(2,2,3)
plot(t,xp_s_delay(1,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,xp_m_delay(1,1:length(t)),'--','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(Velocity x)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\dot{x}_s(t-h_2)$','$\dot{x}_m(t-h_1)$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(2,2,4)
plot(t,xp_s_delay(2,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,xp_m_delay(2,1:length(t)),'--','Color',C16,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[m/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'(Velocity y)'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\dot{y}_s(t-h_1)$','$\dot{y}_m{t-h_1}$'},'interpreter','latex','fontsize',fontsizeLegend)

save("Data_Delay_a_new.mat", "q_s", "q_m", "q_s_delay", "q_m_delay", "x_s", "x_m_base", "x_s_delay", "x_m_base_delay", "xp_s", "xp_m", "xp_s_delay", "xp_m_delay", "he_m", "he_s", "RMSE_x_s_new", "RMSE_y_s_new", "t", "u_cartesian_s", "u_cartesian_m", "x_m_0", "L1_s", "L2_s", "L1_m", "L2_m", "e_new");

