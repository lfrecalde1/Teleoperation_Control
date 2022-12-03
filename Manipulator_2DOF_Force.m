%% Code Manipulator 2DOF
% Clean variables
clc, clear all, close all;

% Time defintion variables
t_s = 0.01;
t_final = 100;
t = (0:t_s:t_final);

g = 9.8;
% System parameters L1 slave
b1_s = 0;
m1_s = 0.8;
l1_s = 1;
Iz1_s= 0.5;

% System parameters L2 slave
b2_s = 0;
m2_s = 0.8;
l2_s = 1;
Iz2_s= 0.5;


L1_s = [b1_s , m1_s, l1_s, Iz1_s];
L2_s = [b2_s , m2_s, l2_s, Iz2_s];

% Initial conditions system slave       
q_s = zeros(4, length(t)+1);
q_s(:, 1) = [200*pi/180;...
            -90*pi/180;...
           0;...
           0];
       
% System parameters L1 slave
b1_m = 0;
m1_m = 0.8;
l1_m = 1;
Iz1_m= 0.5;

% System parameters L2 slave
b2_m = 0;
m2_m = 0.8;
l2_m = 1;
Iz2_m= 0.5;


L1_m = [b1_m , m1_m, l1_m, Iz1_m];
L2_m = [b2_m , m2_m, l2_m, Iz2_m];

% Initial conditions system master       
q_m = zeros(4, length(t)+1);
q_m(:, 1) = [200*pi/180;...
            -90*pi/180;...
           0;...
           0];
       
       
% Constant defintion
constans = [g, t_s];

% Robot definition slave
robot_s = manipulator_system(L1_s, L2_s, constans, q_s(:,1));

% Robot definition Master
robot_m = manipulator_system(L1_m, L2_m, constans, q_m(:,1));

% Inital Position Carterian Space slave
x_s = zeros(2, length(t)+1);
xp_s = zeros(2, length(t)+1);
x_s(:, 1) = robot_s.get_general_position();
xp_s(:, 1) = robot_s.get_general_velocities();

% Inital Position Carterian Space master
x_m = zeros(2, length(t)+1);
xp_m = zeros(2, length(t)+1);
x_m_0 = [2.5; 0];
x_m(:, 1) = x_m_0 + robot_m.get_general_position();
xp_m(:, 1) = robot_m.get_general_velocities();

% Desired angles of the system
qd = [90*pi/180*ones(1, length(t));...
       0*pi/180*ones(1, length(t))];
   
qdp = [0*pi/180*ones(1, length(t));...
       0*pi/180*ones(1, length(t))];
   
qdpp = [0*pi/180*ones(1, length(t));...
        0*pi/180*ones(1, length(t))];

% Desired Position cartesian Space
xd_c = [0.0*sin(0.1*t);...
      -0.1*ones(1, length(t))];
  
xdp_c = [(0.0*0.1)*cos(0.1*t);...
       0*ones(1, length(t))];

xdpp_c = [-(0.0*0.1*0.1)*sin(0.1*t);...
        0*ones(1, length(t))];

% Control gains
kp = 5;
wn = sqrt(kp);
kv = 8*1*wn;

ki = 5;
kp_force = 2;
wn_force = sqrt(kp_force);
kv_force = 20*1*wn_force;
% Learning variable
max_value = (((kv^2)/2)-2)/(sqrt((kv^2)/(4)-1));

gamma = 0.1;

% Learning system
% Transfer funtion definitions
P = tf([1 (kv-gamma) kp-gamma], [0 0 1]);
A = tf([1], [1 kv kp]);
S = tf([0 0 1], [0 1 0]);

% Filter force design
kp_f = 1;
wn_f = sqrt(kp_f);
kv_f = 1.2*1*wn_f;

Filter = tf([1], [1 kv_f kp_f]);
Filter_d = c2d(Filter, t_s);
[num1d_filter, den1d_filter] = tfdata(Filter_d,'v');

% Filter coeficients
A_filter = num1d_filter(2:end);
B_filter = den1d_filter(2:end);

% Operator contraction
aux = S*P*A;

% Operator Function
aux_d = c2d(aux, t_s);
[num1d, den1d] = tfdata(aux_d,'v');


% PD control Gains
K1 = kp*eye(2);
K2 = kv*eye(2);
Km = ki*eye(2);

% Cntrol force gains
KP_force = kp_force*eye(2);
KD_force = kv_force*eye(2);

% Control slave gains
kp_s = 5;
wn_s = sqrt(kp_s);
kv_s = 2.5*1*wn_s;
KP_s= kp_s*eye(2);
KD_s = kv_s*eye(2);

% Control master gains
kp_m = 5;
wn_m = sqrt(kp_m);
kv_m = 2.5*1*wn_m;
KP_m= kp_m*eye(2);
KD_m = kv_m*eye(2);

% Control gains operator
kp_h = 5;
wn_h = sqrt(kp_h);
kv_h = 3.5*1*wn_h;

KP_h = kp_h*eye(2);
KD_h = kv_h*eye(2);

% Controller definition slave
control_s = controller(K1, K2, Km, KP_force, KD_force, KP_s, KD_s, num1d, den1d, robot_s);

u_cartesian_s = zeros(2, length(t));

% Controller definition master
control_m = controller(K1, K2, Km, KP_force, KD_force, KP_m, KD_m, num1d, den1d, robot_m);

u_cartesian_m = zeros(2, length(t));
% Control vector error definition
qe = zeros(2, length(t));
qep = zeros(2, length(t));

xe = zeros(2, length(t));
xep = zeros(2, length(t));


% External Force Reaction
x_enviroment_c = (-0.8:0.01:0.8);
x_enviroment_c(2, :) = 0*ones(1, length(x_enviroment_c));

% Location Desired In I
r_c = [1.5;0.5];
% Angle new Axis
angle_c = 90*pi/180;

% Rotational Matriz
Rot_c = [cos(angle_c), -sin(angle_c);...
         sin(angle_c), cos(angle_c)];

% Desired Trajectory in I
x_enviroment_i = r_c + Rot_c*x_enviroment_c;  

% Enviroment Force
F_enviroment = zeros(2, length(t)+1);


% Potential Field
[V, index] = potential_field_final(x_s(:, 1), x_enviroment_i(:, :));

% Angle Between end effector and Obstacle
beta = angle_obstacle(x_s(:, 1), x_enviroment_i(:, index));

% Force In I
F_enviroment(:, 1) = [cos(beta);sin(beta)]*V;

% Forces Frame C
F_c(:,1) = inv(Rot_c)*F_enviroment(:,1);

% Force Filter
F_x_memory = zeros(length(den1d_filter(2:end)),1);
F_y_memory = zeros(length(den1d_filter(2:end)),1);

F_x_filter =  zeros(length(num1d_filter(2:end)),1);
F_y_filter =  zeros(length(num1d_filter(2:end)),1);

% Deisred Trayectory I frame
xd_i = r_c + Rot_c*xd_c;
xdp_i = Rot_c*xdp_c;
xdpp_i = Rot_c*xdpp_c;

% Desired Forces
F_desired = [4*ones(1, length(t)+1);...
             4*ones(1, length(t)+1)];
         
F_desired_p = [0*ones(1, length(t)+1);...
               0*ones(1, length(t)+1)];
           

% Derivative of the Force
Force_c_filter_p = zeros(2, length(t));

% Delay s time definition
delay_s = 0;
n_frames_s = delay_s/t_s;

% Delay signals slave
q_s_delay = zeros(4, length(t)+1);
x_s_delay = zeros(2, length(t)+1);

% Initial Condition
q_s_delay(:, 1:n_frames_s(1) + 1)  = q_s(:,1)*ones(1,n_frames_s + 1);
x_s_delay(:, 1:n_frames_s(1) +1) = x_s(:, 1)*ones(1,n_frames_s + 1);

% Delay m time definition
delay_m = 0;
n_frames_m = delay_m/t_s;

% Delay signals slave
q_m_delay = zeros(4, length(t)+1);
x_m_delay = zeros(2, length(t)+1);

% Initial Condition
q_m_delay(:, 1:n_frames_m(1) + 1)  = q_m(:,1)*ones(1,n_frames_m + 1);
x_m_delay(:, 1:n_frames_m(1) +1) = x_m(:, 1)*ones(1,n_frames_m + 1);

% Desired Reference Operator


for k = 1:length(t)
    % Disntance Forces 
    [V(:, k), index] = potential_field_final(x_s_delay(:, k), x_enviroment_i(:, :));
    
    %Angle To Obstacle genral Coordinate
    beta(:, k) = angle_obstacle(x_s_delay(:, k), x_enviroment_i(:, index));
    
    %External Force
    F_enviroment(:, k) = [cos(beta(:, k));sin(beta(:, k))]*V(:, k);
    
    %External Force frame C
    F_c(:, k) = inv(Rot_c)*F_enviroment(:, k);
    
    % Filter Force 
    [F_x_filter_k, F_x_filter, F_x_memory] = filter(F_enviroment(1, k), F_x_filter, F_x_memory, A_filter, B_filter);
    [F_y_filter_k, F_y_filter, F_y_memory] = filter(F_enviroment(2, k), F_y_filter, F_y_memory, A_filter, B_filter);
    
    % Force I frame filter
    Force_filter(:, k) = [F_x_filter_k;...
                    F_y_filter_k];
                
    % Force C frame Filter 
    Force_c_filter(:, k) = inv(Rot_c)*Force_filter(:, k);
    
    % Derivative Force 
    if k > 1
        Force_c_filter_p(:, k) = (Force_c_filter(:, k) - Force_c_filter(:, k-1))/t_s;
    end
    
    % Control vector
    qe(:, k) = qd(:, k) - q_s_delay(1:2, k);
    qep(:, k) = qdp(:, k) - q_s_delay(3:4, k);
    
    qe_s(:, k) = q_m_delay(1:2, k) - q_s(1:2, k);
    qe_m(:, k) = q_s_delay(1:2, k) - q_m(1:2, k);
    
    % Position Cartesian Space
    xe(:, k) = xd_i(:, k) - robot_s.get_general_position();
    xep(:, k) = xdp_i(:, k) - robot_s.get_general_velocities();
    
    % Error Human Operator
    qe_h(:, k) = xd_i(:, k) - x_s_delay(:, k);
    
    
    % Control Teleoperation part
    u_cartesian_s(:, k) = control_s.get_control_tele(q_s(:, k), q_m_delay(:, k));
    u_cartesian_m(:, k) = control_m.get_control_tele(q_m(:, k), q_s_delay(:, k));
    
    % Control Operator Desired
    F_h(:, k) = KP_h*(xd_i(:,k) - x_s_delay(:, k)) - KD_h*xp_m(:, k);

    
    % System evolution Slave
    q_s(:, k+1) = robot_s.system_f(u_cartesian_s(:, k), - Force_filter(:, k));
    x_s(:, k+1) = robot_s.get_general_position();
    xp_s(:, k+1) = robot_s.get_general_velocities();
    
    % System evolution master
    q_m(:, k+1) = robot_m.system_f(u_cartesian_m(:, k), -F_h(:, k));
    x_m(:, k+1) = x_m_0 + robot_m.get_general_position();
    xp_m(:, k+1) = robot_m.get_general_velocities();
    
    % Delay system 
     if k + n_frames_s < (length(t)+ 1)
        q_s_delay(:, k + n_frames_s + 1) = q_s(:, k + 1); 
        x_s_delay(:, k + n_frames_s + 1) = x_s(:, k + 1); 
     end
     
     if k + n_frames_m < (length(t)+ 1)
         q_m_delay(:, k + n_frames_m + 1) = q_m(:, k + 1);
         x_m_delay(:, k + n_frames_m + 1) = x_m(:, k + 1);
     end
     

end

for k = 1:10:length(t)
    drawpend2(q_s_delay(:, k), m1_s, m2_s, 0.3, l1_s, l2_s, q_m_delay(:, k), m1_m, m2_m, 0.3, l1_m, l2_m, x_m_0, x_enviroment_i(:, :), xd_i(:,k));
end
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
%plot(uv(1,:),uv(2,:),'-','Color',C11,'LineWidth',lw);
plot(t,q_s_delay(1,1:length(t)),'--','Color',C12,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[rad]$','interpreter','latex','fontsize',fontsizeLabel)
title({'$(Slave)$'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$q_1$','$q_{1_delay}$'},'interpreter','latex','fontsize',fontsizeLegend)
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
title({'$(Slave)$'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$q_2$','$q_{2_delay}$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(2,2,3)
plot(t,q_s(3,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,q_s_delay(3,1:length(t)),'--','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'$(Slave)$'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\dot{q}_1$','$\dot{q}_{1_delay}$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(2,2,4)
plot(t,q_s(4,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,q_s_delay(4,1:length(t)),'--','Color',C16,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'$(Slave)$'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\dot{q}_2$','$\dot{q}_{2_delay}$'},'interpreter','latex','fontsize',fontsizeLegend)


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
title({'$(Master)$'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$q_1$','$q_{1_delay}$'},'interpreter','latex','fontsize',fontsizeLegend)
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
title({'$(Master)$'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$q_2$','$q_{2_delay}$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(2,2,3)
plot(t,q_m(3,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,q_m_delay(3,1:length(t)),'--','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'$(Master)$'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\dot{q}_1$','$\dot{q}_{1_delay}$'},'interpreter','latex','fontsize',fontsizeLegend)

subplot(2,2,4)
plot(t,q_m(4,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,q_m_delay(4,1:length(t)),'--','Color',C16,'LineWidth',lw);
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[rad/s]$','interpreter','latex','fontsize',fontsizeLabel)
title({'$(Master)$'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$\dot{q}_2$','$\dot{q}_{2_delay}$'},'interpreter','latex','fontsize',fontsizeLegend)


figure('Position', [10 10 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');

subplot(1,2,1)
plot(t,qe_m(1,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
%plot(uv(1,:),uv(2,:),'-','Color',C11,'LineWidth',lw);
plot(t,qe_m(2,1:length(t)),'--','Color',C12,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[rad]$','interpreter','latex','fontsize',fontsizeLabel)
title({'$(Master)(q_s(t-h_2)-q_m)$'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$qe_{m1}$','$qe_{m2}$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;


subplot(1,2,2)
plot(t,qe_s(1,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
plot(t,qe_s(2,1:length(t)),'--','Color',C13,'LineWidth',lw);
grid minor;

set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
%xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[rad]$','interpreter','latex','fontsize',fontsizeLabel)
title({'$(Slave)(q_m(t-h_1)- q_s)$'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$qe_{s1}$','$qe_{s2}$'},'interpreter','latex','fontsize',fontsizeLegend)



figure('Position', [10 10 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');

subplot(1,1,1)
plot(t,qe_h(1,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
%plot(uv(1,:),uv(2,:),'-','Color',C11,'LineWidth',lw);
plot(t,qe_h(2,1:length(t)),'--','Color',C12,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[rad]$','interpreter','latex','fontsize',fontsizeLabel)
title({'$(Master)(x_d - x_s(t - h_2))$'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$qe_{h1}$','$qe_{h2}$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;


figure('Position', [10 10 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');

subplot(1,1,1)
plot(t,F_h(1,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
%plot(uv(1,:),uv(2,:),'-','Color',C11,'LineWidth',lw);
plot(t,F_h(2,1:length(t)),'--','Color',C12,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[N]$','interpreter','latex','fontsize',fontsizeLabel)
title({'$Human$'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$F_{h1}$','$F_{h2}$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;

figure('Position', [10 10 sizeX sizeY])
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8.5 11]);
set(gcf, 'PaperPositionMode', 'manual');

subplot(1,1,1)
plot(t,Force_filter(1,1:length(t)),'-','Color',C9,'LineWidth',lw*1.2); hold on
%plot(uv(1,:),uv(2,:),'-','Color',C11,'LineWidth',lw);
plot(t,Force_filter(2,1:length(t)),'--','Color',C12,'LineWidth',lw);
grid minor;
grid minor;
set(gca,'ticklabelinterpreter','latex',...
        'fontsize',fontsizeTicks)
xlabel('$\textrm{Time}[s]$','interpreter','latex','fontsize',fontsizeLabel)
ylabel('$[N]$','interpreter','latex','fontsize',fontsizeLabel)
title({'$Enviroment$'},'fontsize',fontsizeTitel,'interpreter','latex')
% set(gca,'Xticklabel',[])
legend({'$F_{e1}$','$F_{e2}$'},'interpreter','latex','fontsize',fontsizeLegend)
%plot(t,ul,'-','Color',C2,'LineWidth',lw);
grid minor;


save("Data_Tele_delay.mat", "t", "q_s", "qd", "qdp", "qe", "qep", "u_cartesian_s","L1_s", "L2_s", "xd_c", "xdp_c", "x_s", "xp_s", "xe", "xep", "F_enviroment", "V", "beta", "xd_i", "xdp_i", "F_c", "Force_filter", "Force_c_filter", "F_desired", "x_enviroment_i")