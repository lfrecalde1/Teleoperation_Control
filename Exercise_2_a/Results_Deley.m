%% Teleoperation No Delay Results

% clean variables of the system 
clc, clear all, close all;

% Load Data of the system
load("Data_Delay_a_new.mat")

% Change dimentions in the variables
q_m = q_m(:, 1:end-1);
q_m_delay = q_m_delay(:, 1:end-1);
q_s = q_s(:, 1:end-1);
q_s_delay = q_s_delay(:, 1:end-1);
x_m_base = x_m_base(:, 1:end-1);
x_m_base_delay = x_m_base_delay(:, 1:end-1);
x_s = x_s(:, 1:end-1);
x_s_delay = x_s_delay(:, 1:end-1);
xp_m = xp_m(:, 1:end-1);
xp_m_delay = xp_m_delay(:,1:end-1);
xp_s = xp_s(:, 1:end-1);
xp_s_delay = xp_s_delay(:, 1:end-1);

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
x_m_delay_plot = line(t,x_m_base_delay(1,:));
set(x_m_delay_plot, 'LineStyle', '-', 'Color', C12, 'LineWidth', 1.2*lw);
x_s_delay_plot = line(t,x_s_delay(1,:));
set(x_s_delay_plot, 'LineStyle', '--', 'Color', C9, 'LineWidth', lw);


% fig1_comps.p1 = ul_plot;
%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
%xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[m]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_1 = legend([x_s_delay_plot,x_m_delay_plot],{'$x_s(t-h_2)$','$x_m(t-h_1)$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks)
     
%% Figure properties
ax_1 = gca;
ax_1.Box = 'on';
ax_1.BoxStyle = 'full';
ax_1.XTickLabel = [];
ax_1.TickLength = [0.01;0.01];
ax_1.TickDirMode = 'auto';
ax_1.YMinorTick = 'on';
ax_1.XMinorTick = 'on';
ax_1.XMinorGrid = 'on';
ax_1.YMinorGrid = 'on';

%ax_1.MinorGridColor = '#8f8f8f';
ax_1.MinorGridAlpha = 0.15;
ax_1.LineWidth = 0.8;

axes('Position',[0.52 0.6 .42 0.35]);
%% Data generation

y_m_delay_plot = line(t,x_m_base_delay(2,:));
set(y_m_delay_plot, 'LineStyle', '-', 'Color', C13, 'LineWidth', lw*1.2);
y_s_delay_plot = line(t,x_s_delay(2,:));
set(y_s_delay_plot, 'LineStyle', '--', 'Color', C9, 'LineWidth', lw);
% fig1_comps.p1 = ul_plot;
%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
%xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[m]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_2 = legend([y_s_delay_plot,y_m_delay_plot],{'$y_s(t-h_2)$','$y_m(t-h_1)$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks)
     
%% Figure properties
ax_2 = gca;
ax_2.Box = 'on';
ax_2.BoxStyle = 'full';
ax_2.XTickLabel = [];
ax_2.TickLength = [0.01;0.01];
ax_2.TickDirMode = 'auto';
ax_2.YMinorTick = 'on';
ax_2.XMinorTick = 'on';
ax_2.XMinorGrid = 'on';
ax_2.YMinorGrid = 'on';
%ax_1.MinorGridColor = '#8f8f8f';
ax_2.MinorGridAlpha = 0.15;
ax_2.LineWidth = 0.8;

axes('Position',[0.05 0.2 .42 0.35]);
%% Data generation
xp_m_delay_plot = line(t,xp_m_delay(1,:));
set(xp_m_delay_plot, 'LineStyle', '-', 'Color', C2, 'LineWidth', lw*1.2);
xp_s_delay_plot = line(t,xp_s_delay(1,:));
set(xp_s_delay_plot, 'LineStyle', '--', 'Color', C9, 'LineWidth', lw);


% fig1_comps.p1 = ul_plot;
%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[m/s]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_3 = legend([xp_s_delay_plot,xp_m_delay_plot],{'$\dot{x}_s(t-h_2)$','$\dot{x}_m(t-h_1)$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
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
yp_m_delay_plot = line(t,xp_m_delay(2,:));
set(yp_m_delay_plot, 'LineStyle', '-', 'Color', C16, 'LineWidth', lw*1.2);
yp_s_delay_plot = line(t,xp_s_delay(2,:));
set(yp_s_delay_plot, 'LineStyle', '--', 'Color', C9, 'LineWidth', lw);

% fig1_comps.p1 = ul_plot;
%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[m/s]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_4 = legend([yp_s_delay_plot,yp_m_delay_plot],{'$\dot{y}_s(t-h_2)$','$\dot{y}_m(t-h_1)$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
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
export_fig Results_delay_signals_a.pdf -q101

figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;

axes('Position',[0.05 0.6 .42 0.35]);
%% Data generation
u_s_1_plot = line(t,u_cartesian_s(1,:));
set(u_s_1_plot, 'LineStyle', '-', 'Color', C16, 'LineWidth', 1.2*lw);
u_s_2_plot = line(t,u_cartesian_s(2,:));
set(u_s_2_plot, 'LineStyle', '--', 'Color', C9, 'LineWidth', lw);


% fig1_comps.p1 = ul_plot;
%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[N.m]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_5 = legend([u_s_1_plot,u_s_2_plot],{'$^{1}\tau_s$','$^{2}\tau_s$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks)
     
%% Figure properties
ax_5 = gca;
ax_5.Box = 'on';
ax_5.BoxStyle = 'full';
ax_5.TickLength = [0.01;0.01];
ax_5.TickDirMode = 'auto';
ax_5.YMinorTick = 'on';
ax_5.XMinorTick = 'on';
ax_5.XMinorGrid = 'on';
ax_5.YMinorGrid = 'on';

%ax_1.MinorGridColor = '#8f8f8f';
ax_5.MinorGridAlpha = 0.15;
ax_1.LineWidth = 0.8;

axes('Position',[0.52 0.6 .42 0.35]);
%% Data generation

u_m_1_plot = line(t,u_cartesian_m(1,:));
set(u_m_1_plot, 'LineStyle', '-', 'Color', C12, 'LineWidth', 1.2*lw);
u_m_2_plot = line(t,u_cartesian_m(2,:));
set(u_m_2_plot, 'LineStyle', '--', 'Color', C9, 'LineWidth', lw);


% fig1_comps.p1 = ul_plot;
%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[N.m]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_6 = legend([u_m_1_plot,u_m_2_plot],{'$^{1}\tau_m$','$^{2}\tau_m$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
 set(gca,'ticklabelinterpreter','latex',...
         'fontsize',fontsizeTicks)
     
%% Figure properties
ax_6 = gca;
ax_6.Box = 'on';
ax_6.BoxStyle = 'full';
ax_6.TickLength = [0.01;0.01];
ax_6.TickDirMode = 'auto';
ax_6.YMinorTick = 'on';
ax_6.XMinorTick = 'on';
ax_6.XMinorGrid = 'on';
ax_6.YMinorGrid = 'on';

%ax_1.MinorGridColor = '#8f8f8f';
ax_6.MinorGridAlpha = 0.15;
ax_6.LineWidth = 0.8;

set(gcf, 'Color', 'w'); % Sets axes background
export_fig Results_delay_Torques_a.pdf -q101

% Create Figure
figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;

%% Data generation
axes('Position',[0.1 0.2 .4 .7]);
x11_s_t_0 = l1_s*sin(q_s_delay(1, 1));
y11_s_t_0 = -l1_s*cos(q_s_delay(1, 1));

x12_s_t_0 = l2_s*sin(q_s_delay(1, 1) + q_s_delay(2, 1)) + l1_s*sin(q_s_delay(1, 1));
y12_s_t_0= -l2_s*cos(q_s_delay(1, 1) + q_s_delay(2, 1)) - l1_s*cos(q_s_delay(1, 1));

%% Real system plot
l_1_s_real_t_0 = line([0 x11_s_t_0],[0 y11_s_t_0]);
l_2_s_real_t_0 = line([x11_s_t_0 x12_s_t_0],[y11_s_t_0 y12_s_t_0]);


rectangle('Position',[x11_s_t_0-m1_s_r/2 y11_s_t_0-m1_s_r/2 m1_s_r m1_s_r],'Curvature',1,'FaceColor',c3);
rectangle('Position',[x12_s_t_0-m2_s_r/2 y12_s_t_0-m2_s_r/2 m2_s_r m2_s_r],'Curvature',1,'FaceColor',c3);

set(l_1_s_real_t_0, 'LineStyle', '-', 'Color', c3, 'LineWidth', lw*1.5)
set(l_2_s_real_t_0, 'LineStyle', '-', 'Color', c3, 'LineWidth', lw*1.5)


x11_m_t_0 = x_m_0(1) + l1_m*sin(q_m_delay(1, 1));
y11_m_t_0 = x_m_0(2) -l1_m*cos(q_m_delay(1, 1));

x12_m_t_0 = x_m_0(1) + l2_m*sin(q_m_delay(1, 1) + q_m_delay(2, 1)) + l1_m*sin(q_m_delay(1, 1));
y12_m_t_0 = x_m_0(2) -l2_m*cos(q_m_delay(1, 1) + q_m_delay(2, 1)) - l1_m*cos(q_m_delay(1, 1));

%% Real system plot
l_1_m_real_t_0 = line([x_m_0(1) x11_m_t_0],[x_m_0(2) y11_m_t_0]);
l_2_m_real_t_0 = line([x11_m_t_0 x12_m_t_0],[y11_m_t_0 y12_m_t_0]);


rectangle('Position',[x11_m_t_0-m1_m_r/2 y11_m_t_0-m1_m_r/2 m1_m_r m1_m_r],'Curvature',1,'FaceColor',c4);
rectangle('Position',[x12_m_t_0-m2_m_r/2 y12_m_t_0-m2_m_r/2 m2_m_r m2_m_r],'Curvature',1,'FaceColor',c4);

set(l_1_m_real_t_0, 'LineStyle', '-', 'Color', c4, 'LineWidth', lw*1.5)
set(l_2_m_real_t_0, 'LineStyle', '-', 'Color', c4, 'LineWidth', lw*1.5)

hTitle_7 = title({'$t = 0[s]$'},'fontsize',14,'interpreter','latex','Color',C18);
hXLabel_7 = xlabel('$x[m]$','fontsize',10,'interpreter','latex', 'Color',C18);
hYLabel_7 = ylabel('$y[m]$','fontsize',10,'interpreter','latex', 'Color',C18);
%% Legend nomeclature
hLegend_7 = legend([l_1_s_real_t_0, l_1_m_real_t_0],{'$\textrm{Slave}$','$\textrm{Master}$'},'fontsize',10,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
% Figure properties
ax_7 = gca;
ax_7.Box = 'on';
ax_7.BoxStyle = 'full';
ax_7.TickLength = [0.02;0.02];
ax_7.TickDirMode = 'auto';
ax_7.YMinorTick = 'on';
ax_7.XMinorTick = 'on';
ax_7.XMinorGrid = 'on';
ax_7.YMinorGrid = 'on';
ax_7.MinorGridAlpha = 0.15;
ax_7.LineWidth = 0.8;
ax_7.XLim = [-0.5 3.1];
ax_7.YLim = [-1.8 1.8];

axes('Position',[0.55 0.2 .4 .7]);
x11_s_t_f = l1_s*sin(q_s_delay(1, end));
y11_s_t_f = -l1_s*cos(q_s_delay(1, end));

x12_s_t_f = l2_s*sin(q_s_delay(1, end) + q_s_delay(2, end)) + l1_s*sin(q_s_delay(1, end));
y12_s_t_f = -l2_s*cos(q_s_delay(1, end) + q_s_delay(2, end)) - l1_s*cos(q_s_delay(1, end));

%% Real system plot
l_1_s_real_t_f = line([0 x11_s_t_f],[0 y11_s_t_f]);
l_2_s_real_t_f = line([x11_s_t_f x12_s_t_f],[y11_s_t_f y12_s_t_f]);


rectangle('Position',[x11_s_t_f-m1_s_r/2 y11_s_t_f-m1_s_r/2 m1_s_r m1_s_r],'Curvature',1,'FaceColor',c3);
rectangle('Position',[x12_s_t_f-m2_s_r/2 y12_s_t_f-m2_s_r/2 m2_s_r m2_s_r],'Curvature',1,'FaceColor',c3);

set(l_1_s_real_t_f, 'LineStyle', '-', 'Color', c3, 'LineWidth', lw*1.5)
set(l_2_s_real_t_f, 'LineStyle', '-', 'Color', c3, 'LineWidth', lw*1.5)


x11_m_t_f = x_m_0(1) + l1_m*sin(q_m_delay(1, end));
y11_m_t_f = x_m_0(2) -l1_m*cos(q_m_delay(1, end));

x12_m_t_f = x_m_0(1) + l2_m*sin(q_m_delay(1, end) + q_m_delay(2, end)) + l1_m*sin(q_m_delay(1, end));
y12_m_t_f = x_m_0(2) -l2_m*cos(q_m_delay(1, end) + q_m_delay(2, end)) - l1_m*cos(q_m_delay(1, end));

%% Real system plot
l_1_m_real_t_f = line([x_m_0(1) x11_m_t_f],[x_m_0(2) y11_m_t_f]);
l_2_m_real_t_f = line([x11_m_t_f x12_m_t_f],[y11_m_t_f y12_m_t_f]);


rectangle('Position',[x11_m_t_f-m1_m_r/2 y11_m_t_f-m1_m_r/2 m1_m_r m1_m_r],'Curvature',1,'FaceColor',c4);
rectangle('Position',[x12_m_t_f-m2_m_r/2 y12_m_t_f-m2_m_r/2 m2_m_r m2_m_r],'Curvature',1,'FaceColor',c4);

set(l_1_m_real_t_f, 'LineStyle', '-', 'Color', c4, 'LineWidth', lw*1.5)
set(l_2_m_real_t_f, 'LineStyle', '-', 'Color', c4, 'LineWidth', lw*1.5)


time_string_number = string(t(end));
time_string_final = "$t = " + time_string_number + "[ s]$";

hTitle_8 = title({time_string_final},'fontsize',14,'interpreter','latex','Color',C18);
hXLabel_8 = xlabel('$x[m]$','fontsize',10,'interpreter','latex', 'Color',C18);
hYLabel_8 = ylabel('$y[m]$','fontsize',10,'interpreter','latex', 'Color',C18);
%% Legend nomeclature
hLegend_8 = legend([l_1_s_real_t_f, l_1_m_real_t_f],{'$\textrm{Slave}$','$\textrm{Master}$'},'fontsize',10,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
% Figure properties
ax_8 = gca;
ax_8.Box = 'on';
ax_8.BoxStyle = 'full';
ax_8.TickLength = [0.02;0.02];
ax_8.TickDirMode = 'auto';
ax_8.YMinorTick = 'on';
ax_8.XMinorTick = 'on';
ax_8.XMinorGrid = 'on';
ax_8.YMinorGrid = 'on';
ax_8.MinorGridAlpha = 0.15;
ax_8.LineWidth = 0.8;
ax_8.XLim = [-0.5 3.1];
ax_8.YLim = [-1.8 1.8];
set(gcf, 'Color', 'w'); % Sets axes background
export_fig Results_Delay_Links_a.pdf -q101


%% Load Old Data
load("Data_Delay_a_real.mat");
figure('Position', [500 500 sizeX sizeY])
set(gcf, 'Position', [500 500 sizeX sizeY]);
fig1_comps.fig = gcf;

axes('Position',[0.05 0.6 .42 0.35]);
%% Data generation
e_real_plot = line(t,e_real(1,:));
set(e_real_plot, 'LineStyle', '-', 'Color', C16, 'LineWidth', 1.2*lw);
e_new_plot = line(t,e_new(1,:));
set(e_new_plot, 'LineStyle', '--', 'Color', C9, 'LineWidth', lw);


% fig1_comps.p1 = ul_plot;
%% Title of the image
%hTitle_1 = title({'$\textrm{(c)}$'},'fontsize',14,'interpreter','latex','Color',C18);
xlabel('$\textrm{Time}[s]$','fontsize',10,'interpreter','latex','Color',C18);
ylabel('$[Error]$','fontsize',10,'interpreter','latex', 'Color',C18);

%% Legend nomeclature
hLegend_10 = legend([e_new_plot, e_real_plot],{'$||\tilde{\mathbf{x}}||_{Optimizacion}$','$||\tilde{\mathbf{x}}||_{Initial}$'},'fontsize',12,'interpreter','latex','Color',[255 255 255]/255,'Location','best','NumColumns',1,'TextColor','black');
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
ERROR = [RMSE_x_s_new; RMSE_y_s_new];
ERROR = [ERROR,[RMSE_x_s_real; RMSE_y_s_real]];
bar(ERROR);

legend({'$RMSE_{Optimization}$','$RMSE_{Initial}$'},'fontsize',10,'interpreter','latex','Color',[255 255 255]/255,'TextColor','black')

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
export_fig Comparative_results_a.pdf -q101