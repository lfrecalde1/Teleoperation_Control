% Optimization Teleoperation system

% Clean varibales
clc, clear all, close all;
% Gains slave and master

load("Parameters_a.mat");
load("Parameters_b.mat");
load("Parameters_c.mat");
load("Parameters_d.mat");
load("Parameters_e.mat");
load("Parameters_f.mat");
load("Parameters_g.mat");
load("Parameters_h.mat");

% Number experiments
number = 8;
% General vector of the variables Gains No delay
X = [2.41218473361655*ones(1,number);...
    chi_a(1), chi_b(1), chi_c(1), chi_d(1), chi_e(1)+0.5, chi_f(1), chi_g(1), chi_h(1)+0.1;...
    1.96805547864400*ones(1,number);...
    chi_a(2), chi_b(2), chi_c(2), chi_d(2), chi_e(2)+0.5, chi_f(2), chi_g(2), chi_h(2)];

% X = [2.41218473361655*ones(1,number);...
%      3.94499729073964*ones(1,number);...
%      1.96805547864400*ones(1,number);
%      1.05684211781156*ones(1,number)];
% Time definition
t_final = 80;
t_s = 0.01;
t = (0:t_s:t_final);

% Signal Delay a
signal_h1_a = 0.5*ones(1, length(t)+1);
signal_h2_a = 0.5*ones(1, length(t)+1);

signal_h1_a = delay_varying_time(signal_h1_a,t);
signal_h2_a = delay_varying_time(signal_h2_a,t);


% Signal Delay b
signal_h1_b = 0.2*ones(1, length(t)+1);
signal_h2_b = 0.8*ones(1, length(t)+1);

signal_h1_b = delay_varying_time(signal_h1_b,t);
signal_h2_b = delay_varying_time(signal_h2_b,t);

% Signal Delay c
signal_h1_c = 0.8*ones(1, length(t)+1);
signal_h2_c = 0.2*ones(1, length(t)+1);

signal_h1_c = delay_varying_time(signal_h1_c,t);
signal_h2_c = delay_varying_time(signal_h2_c,t);

% Signal Delay d
signal_h1_d = 1*ones(1, length(t)+1);
signal_h2_d = 1*ones(1, length(t)+1);

signal_h1_d = delay_varying_time(signal_h1_d,t);
signal_h2_d = delay_varying_time(signal_h2_d,t);

% Signal Delay e
signal_h1_e =  0.5 + 0.5*sin([t t(end)+t_s]);
signal_h2_e =  0.5 - 0.5*sin([t t(end)+t_s]);

signal_h1_e = delay_varying_time(signal_h1_e,t);
signal_h2_e = delay_varying_time(signal_h2_e,t);

% Signal Delay f
signal_h1_f =  0.5 + 0.2*cos(0.2*[t t(end)+t_s]);
signal_h2_f =  0.5 - 0.2*cos(0.2*[t t(end)+t_s]);

signal_h1_f = delay_varying_time(signal_h1_f,t);
signal_h2_f = delay_varying_time(signal_h2_f,t);

% Signal Delay g
signal_h1_g =  0.5 + signals_generator_sin([t t(end)+t_s],0.5,0.1);
signal_h2_g =  0.5 - signals_generator_sin([t t(end)+t_s],0.5,0.1);

signal_h1_g = delay_varying_time(signal_h1_g,t);
signal_h2_g = delay_varying_time(signal_h2_g,t);

% Signal Delay h
signal_h1_h =  1 + signals_generator_sin([t t(end)+t_s],0.5,0.1);
signal_h2_h =  1 - signals_generator_sin([t t(end)+t_s],0.5,0.1);

signal_h1_h = delay_varying_time(signal_h1_h,t);
signal_h2_h = delay_varying_time(signal_h2_h,t);


% % System delay
h1 = [signal_h1_a;...
      signal_h1_b;...
      signal_h1_c;...
      signal_h1_d;...
      signal_h1_e;...
      signal_h1_f;...
      signal_h1_g;...
      signal_h1_h];
  
h2 = [signal_h2_a;...
      signal_h2_b;...
      signal_h2_c;...
      signal_h2_d;...
      signal_h2_e;...
      signal_h2_f;...
      signal_h2_g;...
      signal_h2_h];


%Phisycal parameters
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
qs = [198*pi/180, 170*pi/180;...
           -110*pi/180, -20*pi/180;...
           0, 0;...
           0, 0];
% Initial Conditions Master
qm = [198*pi/180, 90*pi/180;...
       -130*pi/180, 20*pi/180;...
           0, 0;...
           0, 0];

initial = 1;

% Operator Gains
kp_h = 2;

% Enviroment gains
kp_e = 30;

% init Variables
q_s = zeros(4, length(t)+1, number);
q_m = zeros(4, length(t)+1, number);
q_s_delay = zeros(4, length(t)+1, number);
q_m_delay = zeros(4, length(t)+1, number);
x_s = zeros(2, length(t)+1, number);
x_m_base = zeros(2, length(t)+1, number);
x_s_delay = zeros(2, length(t)+1, number);
x_m_base_delay = zeros(2, length(t)+1, number);
xp_s =  zeros(2, length(t)+1, number);
xp_m = zeros(2, length(t)+1, number);
xp_s_delay =  zeros(2, length(t)+1, number);
xp_m_delay =  zeros(2, length(t)+1, number);
he_m =  zeros(2, length(t), number);
he_s =  zeros(2, length(t), number);
RMSE_x_s_real =  zeros(1, number);
RMSE_y_s_real =  zeros(1, number);
u_cartesian_s = zeros(2, length(t), number);
u_cartesian_m =  zeros(2, length(t), number);
e_real  = zeros(1, length(t), number);
F_h_real = zeros(2, length(t), number);
F_enviroment_real =  zeros(2, length(t)+1, number);

for k = 1:number
    [q_s(:,:,k), q_m(:,:,k), q_s_delay(:,:,k), q_m_delay(:,:,k), x_s(:,:,k), x_m_base(:,:,k), x_s_delay(:,:,k), x_m_base_delay(:,:,k), xp_s(:,:,k), xp_m(:,:,k), xp_s_delay(:,:,k), xp_m_delay(:,:,k), he_m(:,:,k), he_s(:,:,k), RMSE_x_s_real(1,k), RMSE_y_s_real(1,k), u_cartesian_s(:,:,k), u_cartesian_m(:,:,k), x_m_0, e_real(1,:,k), F_h_real(:,:,k), F_enviroment_real(:,:,k), xd_i, x_enviroment_i] = Tele_system_simu(X(:,k), h1(k,:), h2(k,:), t, t_s, L1_s, L2_s, L1_m, L2_m, qs(:, initial), qm(:, initial), kp_h, kp_e);

end

save("Data_Delay_4_real_5.mat", "q_s", "q_m", "q_s_delay", "q_m_delay", "x_s", "x_m_base", "x_s_delay", "x_m_base_delay", "xp_s", "xp_m", "xp_s_delay", "xp_m_delay", "he_m", "he_s", "RMSE_x_s_real", "RMSE_y_s_real", "t", "u_cartesian_s", "u_cartesian_m", "x_m_0", "L1_s", "L2_s", "L1_m", "L2_m", "e_real", "h1", "h2", "F_h_real", "F_enviroment_real", "xd_i", "x_enviroment_i");

