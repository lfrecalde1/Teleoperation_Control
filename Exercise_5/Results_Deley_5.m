%% Teleoperation No Delay Results

% clean variables of the system 
clc, clear all, close all;

% Load Data of the system
load("Data_Delay_5_real_5.mat")

% Change dimentions in the variables
q_m = q_m(:, 1:end-1,:);
q_m_delay = q_m_delay(:, 1:end-1,:);
q_s = q_s(:, 1:end-1,:);
q_s_delay = q_s_delay(:, 1:end-1,:);
x_m_base = x_m_base(:, 1:end-1,:);
x_m_base_delay = x_m_base_delay(:, 1:end-1,:);
x_s = x_s(:, 1:end-1,:);
x_s_delay = x_s_delay(:, 1:end-1,:);
xp_m = xp_m(:, 1:end-1,:);
xp_m_delay = xp_m_delay(:,1:end-1,:);
xp_s = xp_s(:, 1:end-1,:);
xp_s_delay = xp_s_delay(:, 1:end-1,:);

% Parameters system values
% System parameters L1 master
b1_m = L1_m(1);
m1_m = L1_m(2);
l1_m = L1_m(3);
Iz1_m= L1_m(4);

% System parameters L2 master
b2_m = L2_m(1);
m2_m = L2_m(2);
l2_m = L2_m(3);
Iz2_m= L2_m(4);

% Constant values Slave
% System parameters L1 Slave
b1_s = L1_s(1);
m1_s = L1_s(2);
l1_s = L1_s(3);
Iz1_s= L1_s(4);

% System parameters L2 Slave
b2_s = L2_s(1);
m2_s = L2_s(2);
l2_s = L2_s(3);
Iz2_s= L2_s(4);

m1_s_r = 0.05*sqrt(m1_s);
m2_s_r = 0.05*sqrt(m2_s);

m1_m_r = 0.05*sqrt(m1_m);
m2_m_r = 0.05*sqrt(m2_m);

% Colors and size of the letters
lw = 1.5; % linewidth 1
lwV = 2; % linewidth 2
fontsizeLabel = 11; %11
fontsizeLegend = 11;
fontsizeTicks = 11;
fontsizeTitel = 11;
sizeX = 1250; % size figure
sizeY = 550; % size figure

% color propreties
c1 = [80, 81, 79]/255;
c2 = [244, 213, 141]/255;
c3 = [242, 95, 92]/255;
c4 = [112, 141, 129]/255;
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
C18 = [0 0 0];

figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;


axes('Position',[0.05 0.6 .42 0.35]);
%% Data generation
xp_m_delay_plot_a = line(t,xp_m_delay(1,:,1));
set(xp_m_delay_plot_a, 'LineStyle', '-', 'Color', C1, 'LineWidth', lw*1.2);
xp_s_delay_plot_a = line(t,xp_s_delay(1,:,1));
set(xp_s_delay_plot_a, 'LineStyle', '--', 'Color', C2, 'LineWidth', lw);

xp_m_delay_plot_b = line(t,xp_m_delay(1,:,2));
set(xp_m_delay_plot_b, 'LineStyle', '-', 'Color', C3, 'LineWidth', lw*1.2);
xp_s_delay_plot_b = line(t,xp_s_delay(1,:,2));
set(xp_s_delay_plot_b, 'LineStyle', '--', 'Color', C4, 'LineWidth', lw);

xp_m_delay_plot_c = line(t,xp_m_delay(1,:,3));
set(xp_m_delay_plot_c, 'LineStyle', '-', 'Color', C5, 'LineWidth', lw*1.2);
xp_s_delay_plot_c = line(t,xp_s_delay(1,:,3));
set(xp_s_delay_plot_c, 'LineStyle', '--', 'Color', C6, 'LineWidth', lw);

xp_m_delay_plot_d = line(t,xp_m_delay(1,:,4));
set(xp_m_delay_plot_d, 'LineStyle', '-', 'Color', C7, 'LineWidth', lw*1.2);
xp_s_delay_plot_d = line(t,xp_s_delay(1,:,4));
set(xp_s_delay_plot_d, 'LineStyle', '--', 'Color', C8, 'LineWidth', lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[m/s]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_3 = legend([xp_s_delay_plot_a,xp_m_delay_plot_a, xp_s_delay_plot_b,xp_m_delay_plot_b, xp_s_delay_plot_c,xp_m_delay_plot_c, xp_s_delay_plot_d,xp_m_delay_plot_d],{'$^{a}\dot{x}_s(t-h_2)$','$^{a}\dot{x}_m(t-h_1)$','$^{b}\dot{x}_s(t-h_2)$','$^{b}\dot{x}_m(t-h_1)$', '$^{c}\dot{x}_s(t-h_2)$','$^{c}\dot{x}_m(t-h_1)$', '$^{d}\dot{x}_s(t-h_2)$','$^{d}\dot{x}_m(t-h_1)$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',2,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks)
     
%% Figure properties
ax_3 = gca;
ax_3.Box = 'on';
ax_3.BoxStyle = 'full';
%ax_3.XTickLabel = [];
ax_3.TickLength = [0.01;0.01];
ax_3.TickDirMode = 'auto';
ax_3.YMinorTick = 'on';
ax_3.XMinorTick = 'on';
ax_3.XMinorGrid = 'on';
ax_3.YMinorGrid = 'on';

%ax_1.MinorGridColor = '#8f8f8f';
ax_3.MinorGridAlpha = 0.15;
ax_3.LineWidth = 0.8;

axes('Position',[0.52 0.6 .42 0.35]);
%% Data generation
yp_m_delay_plot_a = line(t,xp_m_delay(2,:,1));
set(yp_m_delay_plot_a, 'LineStyle', '-', 'Color', C9, 'LineWidth', lw*1.2);
yp_s_delay_plot_a = line(t,xp_s_delay(2,:,1));
set(yp_s_delay_plot_a, 'LineStyle', '--', 'Color', C10, 'LineWidth', lw);

yp_m_delay_plot_b = line(t,xp_m_delay(2,:,2));
set(yp_m_delay_plot_b, 'LineStyle', '-', 'Color', C11, 'LineWidth', lw*1.2);
yp_s_delay_plot_b = line(t,xp_s_delay(2,:,2));
set(yp_s_delay_plot_b, 'LineStyle', '--', 'Color', C12, 'LineWidth', lw);

yp_m_delay_plot_c = line(t,xp_m_delay(2,:,3));
set(yp_m_delay_plot_c, 'LineStyle', '-', 'Color', C13, 'LineWidth', lw*1.2);
yp_s_delay_plot_c = line(t,xp_s_delay(2,:,3));
set(yp_s_delay_plot_c, 'LineStyle', '--', 'Color', C14, 'LineWidth', lw);

yp_m_delay_plot_d = line(t,xp_m_delay(2,:,4));
set(yp_m_delay_plot_d, 'LineStyle', '-', 'Color', C15, 'LineWidth', lw*1.2);
yp_s_delay_plot_d = line(t,xp_s_delay(2,:,4));
set(yp_s_delay_plot_d, 'LineStyle', '--', 'Color', C16, 'LineWidth', lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[m/s]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_4 = legend([yp_s_delay_plot_a,yp_m_delay_plot_a,yp_s_delay_plot_b,yp_m_delay_plot_b,yp_s_delay_plot_c,yp_m_delay_plot_c,yp_s_delay_plot_d,yp_m_delay_plot_d],{'$^{a}\dot{y}_s(t-h_2)$','$^{a}\dot{y}_m(t-h_1)$','$^{b}\dot{y}_s(t-h_2)$','$^{b}\dot{y}_m(t-h_1)$','$^{c}\dot{y}_s(t-h_2)$','$^{c}\dot{y}_m(t-h_1)$','$^{d}\dot{y}_s(t-h_2)$','$^{d}\dot{y}_m(t-h_1)$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',2,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks)
     
%% Figure properties
ax_4 = gca;
ax_4.Box = 'on';
ax_4.BoxStyle = 'full';
ax_4.TickLength = [0.01;0.01];
ax_4.TickDirMode = 'auto';
ax_4.YMinorTick = 'on';
ax_4.XMinorTick = 'on';
ax_4.XMinorGrid = 'on';
ax_4.YMinorGrid = 'on';
%ax_1.MinorGridColor = '#8f8f8f';
ax_4.MinorGridAlpha = 0.15;
ax_4.LineWidth = 0.8;

axes('Position',[0.05 0.2 .42 0.35]);
%% Data generation
xp_m_delay_plot_e = line(t,xp_m_delay(1,:,5));
set(xp_m_delay_plot_e, 'LineStyle', '-', 'Color', C1, 'LineWidth', lw*1.2);
xp_s_delay_plot_e = line(t,xp_s_delay(1,:,5));
set(xp_s_delay_plot_e, 'LineStyle', '--', 'Color', C2, 'LineWidth', lw);

xp_m_delay_plot_f = line(t,xp_m_delay(1,:,6));
set(xp_m_delay_plot_f, 'LineStyle', '-', 'Color', C3, 'LineWidth', lw*1.2);
xp_s_delay_plot_f = line(t,xp_s_delay(1,:,6));
set(xp_s_delay_plot_f, 'LineStyle', '--', 'Color', C4, 'LineWidth', lw);

xp_m_delay_plot_g = line(t,xp_m_delay(1,:,7));
set(xp_m_delay_plot_g, 'LineStyle', '-', 'Color', C5, 'LineWidth', lw*1.2);
xp_s_delay_plot_g = line(t,xp_s_delay(1,:,7));
set(xp_s_delay_plot_g, 'LineStyle', '--', 'Color', C6, 'LineWidth', lw);

xp_m_delay_plot_h = line(t,xp_m_delay(1,:,8));
set(xp_m_delay_plot_h, 'LineStyle', '-', 'Color', C7, 'LineWidth', lw*1.2);
xp_s_delay_plot_h = line(t,xp_s_delay(1,:,8));
set(xp_s_delay_plot_h, 'LineStyle', '--', 'Color', C8, 'LineWidth', lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[m/s]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_3 = legend([xp_s_delay_plot_e,xp_m_delay_plot_e, xp_s_delay_plot_f,xp_m_delay_plot_f, xp_s_delay_plot_g,xp_m_delay_plot_g, xp_s_delay_plot_h,xp_m_delay_plot_h],{'$^{e}\dot{x}_s(t-h_2)$','$^{e}\dot{x}_m(t-h_1)$','$^{f}\dot{x}_s(t-h_2)$','$^{f}\dot{x}_m(t-h_1)$', '$^{g}\dot{x}_s(t-h_2)$','$^{g}\dot{x}_m(t-h_1)$', '$^{h}\dot{x}_s(t-h_2)$','$^{h}\dot{x}_m(t-h_1)$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',2,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks)
     
%% Figure properties
ax_3 = gca;
ax_3.Box = 'on';
ax_3.BoxStyle = 'full';
%ax_3.XTickLabel = [];
ax_3.TickLength = [0.01;0.01];
ax_3.TickDirMode = 'auto';
ax_3.YMinorTick = 'on';
ax_3.XMinorTick = 'on';
ax_3.XMinorGrid = 'on';
ax_3.YMinorGrid = 'on';

%ax_1.MinorGridColor = '#8f8f8f';
ax_3.MinorGridAlpha = 0.15;
ax_3.LineWidth = 0.8;

axes('Position',[0.52 0.2 .42 0.35]);
%% Data generation
yp_m_delay_plot_e = line(t,xp_m_delay(2,:,5));
set(yp_m_delay_plot_e, 'LineStyle', '-', 'Color', C9, 'LineWidth', lw*1.2);
yp_s_delay_plot_e = line(t,xp_s_delay(2,:,5));
set(yp_s_delay_plot_e, 'LineStyle', '--', 'Color', C10, 'LineWidth', lw);

yp_m_delay_plot_f = line(t,xp_m_delay(2,:,6));
set(yp_m_delay_plot_f, 'LineStyle', '-', 'Color', C11, 'LineWidth', lw*1.2);
yp_s_delay_plot_f = line(t,xp_s_delay(2,:,6));
set(yp_s_delay_plot_f, 'LineStyle', '--', 'Color', C12, 'LineWidth', lw);

yp_m_delay_plot_g = line(t,xp_m_delay(2,:,7));
set(yp_m_delay_plot_g, 'LineStyle', '-', 'Color', C13, 'LineWidth', lw*1.2);
yp_s_delay_plot_g = line(t,xp_s_delay(2,:,7));
set(yp_s_delay_plot_g, 'LineStyle', '--', 'Color', C14, 'LineWidth', lw);

yp_m_delay_plot_h = line(t,xp_m_delay(2,:,8));
set(yp_m_delay_plot_h, 'LineStyle', '-', 'Color', C15, 'LineWidth', lw*1.2);
yp_s_delay_plot_h = line(t,xp_s_delay(2,:,8));
set(yp_s_delay_plot_h, 'LineStyle', '--', 'Color', C16, 'LineWidth', lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[m/s]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_4 = legend([yp_s_delay_plot_e,yp_m_delay_plot_e,yp_s_delay_plot_f,yp_m_delay_plot_f,yp_s_delay_plot_g,yp_m_delay_plot_g,yp_s_delay_plot_h,yp_m_delay_plot_h],{'$^{e}\dot{y}_s(t-h_2)$','$^{e}\dot{y}_m(t-h_1)$','$^{f}\dot{y}_s(t-h_2)$','$^{f}\dot{y}_m(t-h_1)$','$^{g}\dot{y}_s(t-h_2)$','$^{g}\dot{y}_m(t-h_1)$','$^{h}\dot{y}_s(t-h_2)$','$^{h}\dot{y}_m(t-h_1)$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',2,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks)
     
%% Figure properties
ax_4 = gca;
ax_4.Box = 'on';
ax_4.BoxStyle = 'full';
ax_4.TickLength = [0.01;0.01];
ax_4.TickDirMode = 'auto';
ax_4.YMinorTick = 'on';
ax_4.XMinorTick = 'on';
ax_4.XMinorGrid = 'on';
ax_4.YMinorGrid = 'on';
%ax_1.MinorGridColor = '#8f8f8f';
ax_4.MinorGridAlpha = 0.15;
ax_4.LineWidth = 0.8;

set(gcf, 'Color', 'w'); % Sets axes background
export_fig Results_delay_signals_5_5.pdf -q101

figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;

axes('Position',[0.05 0.6 .42 0.35]);
%% Data generation
Fh_1_plot_a = line(t,F_h_real(1,:,1));
set(Fh_1_plot_a, 'LineStyle', '-', 'Color', C9, 'LineWidth', 1.2*lw);
Fh_2_plot_a = line(t,F_h_real(2,:,1));
set(Fh_2_plot_a, 'LineStyle', '--', 'Color', C10, 'LineWidth', lw);

Fh_1_plot_b = line(t,F_h_real(1,:,2));
set(Fh_1_plot_b, 'LineStyle', '-', 'Color', C11, 'LineWidth', 1.2*lw);
Fh_2_plot_b = line(t,F_h_real(2,:,2));
set(Fh_2_plot_b, 'LineStyle', '--', 'Color', C12, 'LineWidth', lw);

Fh_1_plot_c = line(t,F_h_real(1,:,3));
set(Fh_1_plot_c, 'LineStyle', '-', 'Color', C13, 'LineWidth', 1.2*lw);
Fh_2_plot_c = line(t,F_h_real(2,:,3));
set(Fh_2_plot_c, 'LineStyle', '--', 'Color', C14, 'LineWidth', lw);

Fh_1_plot_d = line(t,F_h_real(1,:,4));
set(Fh_1_plot_d, 'LineStyle', '-', 'Color', C15, 'LineWidth', 1.2*lw);
Fh_2_plot_d = line(t,F_h_real(2,:,4));
set(Fh_2_plot_d, 'LineStyle', '--', 'Color', C16, 'LineWidth', lw);
% fig1_comps.p1 = ul_plot;

%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[N]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_9 = legend([Fh_1_plot_a,Fh_2_plot_a,Fh_1_plot_b,Fh_2_plot_b,Fh_1_plot_c,Fh_2_plot_c,Fh_1_plot_d,Fh_2_plot_d],{'$^{a}F_{hx}$','$^{a}F_{hy}$','$^{b}F_{hx}$','$^{b}F_{hy}$','$^{c}F_{hx}$','$^{c}F_{hy}$','$^{d}F_{hx}$','$^{d}F_{hy}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',2,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks)
     
%% Figure properties
ax_9 = gca;
ax_9.Box = 'on';
ax_9.BoxStyle = 'full';
ax_9.TickLength = [0.01;0.01];
ax_9.TickDirMode = 'auto';
ax_9.YMinorTick = 'on';
ax_9.XMinorTick = 'on';
ax_9.XMinorGrid = 'on';
ax_9.YMinorGrid = 'on';

%ax_1.MinorGridColor = '#8f8f8f';
ax_9.MinorGridAlpha = 0.15;
ax_9.LineWidth = 0.8;

axes('Position',[0.05 0.2 .42 0.35]);

Fh_1_plot_e = line(t,F_h_real(1,:,5));
set(Fh_1_plot_e, 'LineStyle', '-', 'Color', C9, 'LineWidth', 1.2*lw);
Fh_2_plot_e = line(t,F_h_real(2,:,5));
set(Fh_2_plot_e, 'LineStyle', '--', 'Color', C10, 'LineWidth', lw);

Fh_1_plot_f = line(t,F_h_real(1,:,6));
set(Fh_1_plot_f, 'LineStyle', '-', 'Color', C11, 'LineWidth', 1.2*lw);
Fh_2_plot_f = line(t,F_h_real(2,:,6));
set(Fh_2_plot_f, 'LineStyle', '--', 'Color', C12, 'LineWidth', lw);

Fh_1_plot_g = line(t,F_h_real(1,:,7));
set(Fh_1_plot_g, 'LineStyle', '-', 'Color', C13, 'LineWidth', 1.2*lw);
Fh_2_plot_g = line(t,F_h_real(2,:,7));
set(Fh_2_plot_g, 'LineStyle', '--', 'Color', C14, 'LineWidth', lw);

Fh_1_plot_h = line(t,F_h_real(1,:,8));
set(Fh_1_plot_h, 'LineStyle', '-', 'Color', C15, 'LineWidth', 1.2*lw);
Fh_2_plot_h = line(t,F_h_real(2,:,8));
set(Fh_2_plot_h, 'LineStyle', '--', 'Color', C16, 'LineWidth', lw);
% fig1_comps.p1 = ul_plot;
%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[N]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_9 = legend([Fh_1_plot_e,Fh_2_plot_e,Fh_1_plot_f,Fh_2_plot_f,Fh_1_plot_g,Fh_2_plot_g,Fh_1_plot_h,Fh_2_plot_h],{'$^{e}F_{hx}$','$^{e}F_{hy}$','$^{f}F_{hx}$','$^{f}F_{hy}$','$^{g}F_{hx}$','$^{g}F_{hy}$','$^{h}F_{hx}$','$^{h}F_{hy}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',2,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks)
     
%% Figure properties
ax_9 = gca;
ax_9.Box = 'on';
ax_9.BoxStyle = 'full';
ax_9.TickLength = [0.01;0.01];
ax_9.TickDirMode = 'auto';
ax_9.YMinorTick = 'on';
ax_9.XMinorTick = 'on';
ax_9.XMinorGrid = 'on';
ax_9.YMinorGrid = 'on';

%ax_1.MinorGridColor = '#8f8f8f';
ax_9.MinorGridAlpha = 0.15;
ax_9.LineWidth = 0.8;

axes('Position',[0.52 0.6 .42 0.35]);
%% Data generation

F_e_1_plot_a = line(t,F_enviroment_real(1,1:length(t),1));
set(F_e_1_plot_a, 'LineStyle', '-', 'Color', C9, 'LineWidth', 1.2*lw);
F_e_2_plot_a = line(t,F_enviroment_real(2,1:length(t),1));
set(F_e_2_plot_a, 'LineStyle', '--', 'Color', C10, 'LineWidth', lw);

F_e_1_plot_b = line(t,F_enviroment_real(1,1:length(t),2));
set(F_e_1_plot_b, 'LineStyle', '-', 'Color', C11, 'LineWidth', 1.2*lw);
F_e_2_plot_b = line(t,F_enviroment_real(2,1:length(t),2));
set(F_e_2_plot_b, 'LineStyle', '--', 'Color', C12, 'LineWidth', lw);

F_e_1_plot_c = line(t,F_enviroment_real(1,1:length(t),3));
set(F_e_1_plot_c, 'LineStyle', '-', 'Color', C13, 'LineWidth', 1.2*lw);
F_e_2_plot_c = line(t,F_enviroment_real(2,1:length(t),3));
set(F_e_2_plot_c, 'LineStyle', '--', 'Color', C14, 'LineWidth', lw);

F_e_1_plot_d = line(t,F_enviroment_real(1,1:length(t),4));
set(F_e_1_plot_d, 'LineStyle', '-', 'Color', C15, 'LineWidth', 1.2*lw);
F_e_2_plot_d = line(t,F_enviroment_real(2,1:length(t),4));
set(F_e_2_plot_d, 'LineStyle', '--', 'Color', C16, 'LineWidth', lw);


% fig1_comps.p1 = ul_plot;
%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[N]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_6 = legend([F_e_1_plot_a,F_e_2_plot_a,F_e_1_plot_b,F_e_2_plot_b,F_e_1_plot_c,F_e_2_plot_c,F_e_1_plot_d,F_e_2_plot_d],{'$^{a}F_{ex}$','$^{a}F_{ey}$','$^{b}F_{ex}$','$^{b}F_{ey}$','$^{c}F_{ex}$','$^{c}F_{ey}$','$^{d}F_{ex}$','$^{d}F_{ey}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',2,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks)
     
%% Figure properties
ax_10 = gca;
ax_10.Box = 'on';
ax_10.BoxStyle = 'full';
ax_10.TickLength = [0.01;0.01];
ax_10.TickDirMode = 'auto';
ax_10.YMinorTick = 'on';
ax_10.XMinorTick = 'on';
ax_10.XMinorGrid = 'on';
ax_10.YMinorGrid = 'on';

%ax_1.MinorGridColor = '#8f8f8f';
ax_10.MinorGridAlpha = 0.15;
ax_10.LineWidth = 0.8;

axes('Position',[0.52 0.2 .42 0.35]);
%% Data generation

F_e_1_plot_e = line(t,F_enviroment_real(1,1:length(t),5));
set(F_e_1_plot_e, 'LineStyle', '-', 'Color', C9, 'LineWidth', 1.2*lw);
F_e_2_plot_e = line(t,F_enviroment_real(2,1:length(t),5));
set(F_e_2_plot_e, 'LineStyle', '--', 'Color', C10, 'LineWidth', lw);

F_e_1_plot_f = line(t,F_enviroment_real(1,1:length(t),6));
set(F_e_1_plot_f, 'LineStyle', '-', 'Color', C11, 'LineWidth', 1.2*lw);
F_e_2_plot_f = line(t,F_enviroment_real(2,1:length(t),6));
set(F_e_2_plot_f, 'LineStyle', '--', 'Color', C12, 'LineWidth', lw);

F_e_1_plot_g = line(t,F_enviroment_real(1,1:length(t),7));
set(F_e_1_plot_g, 'LineStyle', '-', 'Color', C13, 'LineWidth', 1.2*lw);
F_e_2_plot_g = line(t,F_enviroment_real(2,1:length(t),7));
set(F_e_2_plot_g, 'LineStyle', '--', 'Color', C14, 'LineWidth', lw);

F_e_1_plot_h = line(t,F_enviroment_real(1,1:length(t),8));
set(F_e_1_plot_h, 'LineStyle', '-', 'Color', C15, 'LineWidth', 1.2*lw);
F_e_2_plot_h = line(t,F_enviroment_real(2,1:length(t),8));
set(F_e_2_plot_h, 'LineStyle', '--', 'Color', C16, 'LineWidth', lw);


% fig1_comps.p1 = ul_plot;
%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[N]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_6 = legend([F_e_1_plot_e,F_e_2_plot_e,F_e_1_plot_f,F_e_2_plot_f,F_e_1_plot_g,F_e_2_plot_g,F_e_1_plot_h,F_e_2_plot_h],{'$^{e}F_{ex}$','$^{e}F_{ey}$','$^{f}F_{ex}$','$^{f}F_{ey}$','$^{g}F_{ex}$','$^{g}F_{ey}$','$^{h}F_{ex}$','$^{h}F_{ey}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',2,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks)
     
%% Figure properties
ax_10 = gca;
ax_10.Box = 'on';
ax_10.BoxStyle = 'full';
ax_10.TickLength = [0.01;0.01];
ax_10.TickDirMode = 'auto';
ax_10.YMinorTick = 'on';
ax_10.XMinorTick = 'on';
ax_10.XMinorGrid = 'on';
ax_10.YMinorGrid = 'on';

%ax_1.MinorGridColor = '#8f8f8f';
ax_10.MinorGridAlpha = 0.15;
ax_10.LineWidth = 0.8;


set(gcf, 'Color', 'w'); % Sets axes background
export_fig Results_delay_Forces_5_5.pdf -q101


figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;

axes('Position',[0.05 0.6 .42 0.35]);
%% Data generation
Fhe_1_plot_a = line(t,F_h_real(1,:,1)-F_enviroment_real(1,1:length(t),1));
set(Fhe_1_plot_a, 'LineStyle', '-', 'Color', C9, 'LineWidth', 1.2*lw);

Fhe_1_plot_b = line(t,F_h_real(1,:,2)-F_enviroment_real(1,1:length(t),2));
set(Fhe_1_plot_b, 'LineStyle', '-', 'Color', C10, 'LineWidth', 1.2*lw);

Fhe_1_plot_c = line(t,F_h_real(1,:,3)-F_enviroment_real(1,1:length(t),3));
set(Fhe_1_plot_c, 'LineStyle', '-', 'Color', C13, 'LineWidth', 1.2*lw);

Fhe_1_plot_d = line(t,F_h_real(1,:,4)-F_enviroment_real(1,1:length(t),4));
set(Fhe_1_plot_d, 'LineStyle', '-', 'Color', C14, 'LineWidth', 1.2*lw);


%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[N]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_5 = legend([Fhe_1_plot_a, Fhe_1_plot_b, Fhe_1_plot_c, Fhe_1_plot_d],{'$^{a}\tilde{f}_x$','$^{b}\tilde{f}_x$','$^{c}\tilde{f}_x$','$^{d}\tilde{f}_x$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',2,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks);
     
%% Figure properties
ax_9 = gca;
ax_9.Box = 'on';
ax_9.BoxStyle = 'full';
ax_9.TickLength = [0.01;0.01];
ax_9.TickDirMode = 'auto';
ax_9.YMinorTick = 'on';
ax_9.XMinorTick = 'on';
ax_9.XMinorGrid = 'on';
ax_9.YMinorGrid = 'on';

%ax_1.MinorGridColor = '#8f8f8f';
ax_9.MinorGridAlpha = 0.15;
ax_9.LineWidth = 0.8;

axes('Position',[0.52 0.6 .42 0.35]);
%% Data generation

Fhe_2_plot_a  = line(t,F_h_real(2,:,1)-F_enviroment_real(2,1:length(t),1));
set(Fhe_2_plot_a , 'LineStyle', '-', 'Color', C9, 'LineWidth', 1.2*lw);


Fhe_2_plot_b  = line(t,F_h_real(2,:,2)-F_enviroment_real(2,1:length(t),2));
set(Fhe_2_plot_b , 'LineStyle', '-', 'Color', C10, 'LineWidth', 1.2*lw);


Fhe_2_plot_c  = line(t,F_h_real(2,:,3)-F_enviroment_real(2,1:length(t),3));
set(Fhe_2_plot_c , 'LineStyle', '-', 'Color', C13, 'LineWidth', 1.2*lw);


Fhe_2_plot_d  = line(t,F_h_real(2,:,4)-F_enviroment_real(2,1:length(t),4));
set(Fhe_2_plot_d , 'LineStyle', '-', 'Color', C14, 'LineWidth', 1.2*lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[N]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_6 = legend([Fhe_2_plot_a, Fhe_2_plot_b, Fhe_2_plot_c, Fhe_2_plot_d],{'$^{a}\tilde{f}_y$','$^{b}\tilde{f}_y$', '$^{c}\tilde{f}_y$', '$^{d}\tilde{f}_y$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',2,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks)
     
%% Figure properties
ax_10 = gca;
ax_10.Box = 'on';
ax_10.BoxStyle = 'full';
ax_10.TickLength = [0.01;0.01];
ax_10.TickDirMode = 'auto';
ax_10.YMinorTick = 'on';
ax_10.XMinorTick = 'on';
ax_10.XMinorGrid = 'on';
ax_10.YMinorGrid = 'on';

%ax_1.MinorGridColor = '#8f8f8f';
ax_10.MinorGridAlpha = 0.15;
ax_10.LineWidth = 0.8;

axes('Position',[0.05 0.2 .42 0.35]);
%% Data generation
Fhe_1_plot_e = line(t,F_h_real(1,:,5)-F_enviroment_real(1,1:length(t),5));
set(Fhe_1_plot_e, 'LineStyle', '-', 'Color', C9, 'LineWidth', 1.2*lw);

Fhe_1_plot_f = line(t,F_h_real(1,:,6)-F_enviroment_real(1,1:length(t),6));
set(Fhe_1_plot_f, 'LineStyle', '-', 'Color', C10, 'LineWidth', 1.2*lw);

Fhe_1_plot_g = line(t,F_h_real(1,:,7)-F_enviroment_real(1,1:length(t),7));
set(Fhe_1_plot_g, 'LineStyle', '-', 'Color', C13, 'LineWidth', 1.2*lw);

Fhe_1_plot_h = line(t,F_h_real(1,:,8)-F_enviroment_real(1,1:length(t),8));
set(Fhe_1_plot_h, 'LineStyle', '-', 'Color', C14, 'LineWidth', 1.2*lw);


%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[N]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_5 = legend([Fhe_1_plot_e, Fhe_1_plot_f, Fhe_1_plot_g, Fhe_1_plot_h],{'$^{e}\tilde{f}_x$','$^{f}\tilde{f}_x$','$^{g}\tilde{f}_x$','$^{h}\tilde{f}_x$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',2,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks);
     
%% Figure properties
ax_9 = gca;
ax_9.Box = 'on';
ax_9.BoxStyle = 'full';
ax_9.TickLength = [0.01;0.01];
ax_9.TickDirMode = 'auto';
ax_9.YMinorTick = 'on';
ax_9.XMinorTick = 'on';
ax_9.XMinorGrid = 'on';
ax_9.YMinorGrid = 'on';

%ax_1.MinorGridColor = '#8f8f8f';
ax_9.MinorGridAlpha = 0.15;
ax_9.LineWidth = 0.8;

axes('Position',[0.52 0.2 .42 0.35]);
%% Data generation

Fhe_2_plot_e  = line(t,F_h_real(2,:,5)-F_enviroment_real(2,1:length(t),5));
set(Fhe_2_plot_e , 'LineStyle', '-', 'Color', C9, 'LineWidth', 1.2*lw);


Fhe_2_plot_f  = line(t,F_h_real(2,:,6)-F_enviroment_real(2,1:length(t),6));
set(Fhe_2_plot_f , 'LineStyle', '-', 'Color', C10, 'LineWidth', 1.2*lw);


Fhe_2_plot_g  = line(t,F_h_real(2,:,7)-F_enviroment_real(2,1:length(t),7));
set(Fhe_2_plot_g , 'LineStyle', '-', 'Color', C13, 'LineWidth', 1.2*lw);


Fhe_2_plot_h  = line(t,F_h_real(2,:,8)-F_enviroment_real(2,1:length(t),8));
set(Fhe_2_plot_h , 'LineStyle', '-', 'Color', C14, 'LineWidth', 1.2*lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[N]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_6 = legend([Fhe_2_plot_e, Fhe_2_plot_f, Fhe_2_plot_g, Fhe_2_plot_h],{'$^{e}\tilde{f}_y$','$^{f}\tilde{f}_y$', '$^{g}\tilde{f}_y$', '$^{h}\tilde{f}_y$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',2,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks)
     
%% Figure properties
ax_10 = gca;
ax_10.Box = 'on';
ax_10.BoxStyle = 'full';
ax_10.TickLength = [0.01;0.01];
ax_10.TickDirMode = 'auto';
ax_10.YMinorTick = 'on';
ax_10.XMinorTick = 'on';
ax_10.XMinorGrid = 'on';
ax_10.YMinorGrid = 'on';

%ax_1.MinorGridColor = '#8f8f8f';
ax_10.MinorGridAlpha = 0.15;
ax_10.LineWidth = 0.8;

set(gcf, 'Color', 'w'); % Sets axes background
export_fig Results_delay_Error_Forces_5_5.pdf -q101


%% Load Old Data
figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;

axes('Position',[0.05 0.6 .42 0.35]);
%% Data generation
e_real_plot_a = line(t,e_real(1,:,1));
set(e_real_plot_a, 'LineStyle', '-', 'Color', C9, 'LineWidth', 1.2*lw);

e_real_plot_b = line(t,e_real(1,:,2));
set(e_real_plot_b, 'LineStyle', '-', 'Color', C10, 'LineWidth', 1.2*lw);

e_real_plot_c = line(t,e_real(1,:,3));
set(e_real_plot_c, 'LineStyle', '-', 'Color', C13, 'LineWidth', 1.2*lw);

e_real_plot_d = line(t,e_real(1,:,4));
set(e_real_plot_d, 'LineStyle', '-', 'Color', C14, 'LineWidth', 1.2*lw);


%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[Error]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_10 = legend([e_real_plot_a, e_real_plot_b, e_real_plot_c, e_real_plot_d],{'$||\tilde{\mathbf{x}}||_{a}$','$||\tilde{\mathbf{x}}||_{b}$','$||\tilde{\mathbf{x}}||_{c}$','$||\tilde{\mathbf{x}}||_{d}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',2,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks)

     
%% Figure properties
ax_10 = gca;
ax_10.Box = 'on';
ax_10.BoxStyle = 'full';
ax_10.TickLength = [0.01;0.01];
ax_10.TickDirMode = 'auto';
ax_10.YMinorTick = 'on';
ax_10.XMinorTick = 'on';
ax_10.XMinorGrid = 'on';
ax_10.YMinorGrid = 'on';

%ax_1.MinorGridColor = '#8f8f8f';
ax_10.MinorGridAlpha = 0.15;
ax_10.LineWidth = 0.8;

axes('Position',[0.05 0.2 .42 0.35]);
%% Data generation
e_real_plot_e = line(t,e_real(1,:,5));
set(e_real_plot_e, 'LineStyle', '-', 'Color', C9, 'LineWidth', 1.2*lw);

e_real_plot_f = line(t,e_real(1,:,6));
set(e_real_plot_f, 'LineStyle', '-', 'Color', C10, 'LineWidth', 1.2*lw);

e_real_plot_g = line(t,e_real(1,:,7));
set(e_real_plot_g, 'LineStyle', '-', 'Color', C13, 'LineWidth', 1.2*lw);

e_real_plot_h = line(t,e_real(1,:,8));
set(e_real_plot_h, 'LineStyle', '-', 'Color', C14, 'LineWidth', 1.2*lw);


%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[Error]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_10 = legend([e_real_plot_e, e_real_plot_f, e_real_plot_g, e_real_plot_h],{'$||\tilde{\mathbf{x}}||_{e}$','$||\tilde{\mathbf{x}}||_{f}$','$||\tilde{\mathbf{x}}||_{g}$','$||\tilde{\mathbf{x}}||_{h}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',2,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks)

     
%% Figure properties
ax_10 = gca;
ax_10.Box = 'on';
ax_10.BoxStyle = 'full';
ax_10.TickLength = [0.01;0.01];
ax_10.TickDirMode = 'auto';
ax_10.YMinorTick = 'on';
ax_10.XMinorTick = 'on';
ax_10.XMinorGrid = 'on';
ax_10.YMinorGrid = 'on';

%ax_1.MinorGridColor = '#8f8f8f';
ax_10.MinorGridAlpha = 0.15;
ax_10.LineWidth = 0.8;
x = [1:1:2];

axes('Position',[0.55 0.6 .42 0.35]);

ERROR = [RMSE_x_s_real(1,1); RMSE_y_s_real(1,1)];

ERROR = [ERROR,[RMSE_x_s_real(1,2); RMSE_y_s_real(1,2)]];

ERROR = [ERROR,[RMSE_x_s_real(1,3); RMSE_y_s_real(1,3)]];

ERROR = [ERROR,[RMSE_x_s_real(1,4); RMSE_y_s_real(1,4)]];

bar(ERROR);

legend({'$RMSE_{a}$','$RMSE_{b}$', '$RMSE_{c}$', '$RMSE_{d}$'},'fontsize',10,'interpreter','latex','Color',[255 255 255]/255,'TextColor','black')

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
set(gca,'XTick',[1 2],'XTickLabel',{'$x$','$y$'});

hYLabel_9 = ylabel('$\textrm{RMSE}$','fontsize',10,'interpreter','latex', 'Color',C18);

% Figure properties
ax_9 = gca;
ax_9.Box = 'on';
ax_9.BoxStyle = 'full';
ax_9.TickLength = [0.005;0.005];
ax_9.TickDirMode = 'auto';
ax_9.YMinorTick = 'on';
ax_9.XMinorTick = 'on';
% ax_2.XTickLabel = [];
ax_9.XMinorGrid = 'on';
ax_9.YMinorGrid = 'on';
%ax92.MinorGridColor = '#8f8f8f';
ax_9.MinorGridAlpha = 0.15;
ax_9.YLabel = hYLabel_9;
%ax_9.XLim = [0.8 4.2];
ax_9.LineWidth = 0.8;

axes('Position',[0.55 0.2 .42 0.35]);

ERROR = [RMSE_x_s_real(1,5); RMSE_y_s_real(1,5)];
ERROR = [ERROR,[RMSE_x_s_real(1,6); RMSE_y_s_real(1,6)]];
ERROR = [ERROR,[RMSE_x_s_real(1,7); RMSE_y_s_real(1,7)]];
ERROR = [ERROR,[RMSE_x_s_real(1,8); RMSE_y_s_real(1,8)]];

bar(ERROR);

legend({'$RMSE_{e}$','$RMSE_{f}$', '$RMSE_{g}$', '$RMSE_{h}$'},'fontsize',10,'interpreter','latex','Color',[255 255 255]/255,'TextColor','black')

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
set(gca,'XTick',[1 2],'XTickLabel',{'$x$','$y$'});

hYLabel_9 = ylabel('$\textrm{RMSE}$','fontsize',10,'interpreter','latex', 'Color',C18);

% Figure properties
ax_9 = gca;
ax_9.Box = 'on';
ax_9.BoxStyle = 'full';
ax_9.TickLength = [0.005;0.005];
ax_9.TickDirMode = 'auto';
ax_9.YMinorTick = 'on';
ax_9.XMinorTick = 'on';
% ax_2.XTickLabel = [];
ax_9.XMinorGrid = 'on';
ax_9.YMinorGrid = 'on';
%ax92.MinorGridColor = '#8f8f8f';
ax_9.MinorGridAlpha = 0.15;
ax_9.YLabel = hYLabel_9;
%ax_9.XLim = [0.8 4.2];
ax_9.LineWidth = 0.8;

set(gcf, 'Color', 'w'); % Sets axes background
export_fig Comparative_results_5_5.pdf -q101
