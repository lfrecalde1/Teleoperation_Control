% Solve Optimization gains problem
% Optimization Teleoperation system

% Clean varibales
clc, clear all, close all;
% Gains slave and master

% General vector of the variables Gains No delay
X = [1;1];
XP = [2.41218473361655;1.96805547864400];

% Time definition
t_final = 40;
t_s = 0.01;
t = (0:t_s:t_final);

signal_h1 = 0.5 + 0.5*sin([t t(end)+t_s]);
signal_h2 = 0.5 + 0.5*cos([t t(end)+t_s]);

signal_h1 = delay_varying_time(signal_h1,t);
signal_h2 = delay_varying_time(signal_h2,t);

h1_max = max(signal_h1);
h2_max = max(signal_h2);

% System delay
h1 = h1_max*ones(1, length(t) + 1);
h2 = h2_max*ones(1, length(t) + 1);

% Phisycal parameters
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
qs = [180*pi/180;...
           -30*pi/180;...
           0;...
           0];
       
% Initial Conditions Master
qm = [80*pi/180;...
             10*pi/180;...
           0;...
           0];
% Desired RMES
RMSE_xd = 0.09;
RMSE_yd = 0.095;

% Optimization parameters
options = optimset('Display','iter',...
                'TolFun', 1e-6,...
                'MaxIter', 5,...
                'Algorithm', 'active-set',...
                'FinDiffType', 'forward',...
                'RelLineSrchBnd', [],...
                'RelLineSrchBndDuration', 1,...
                'TolConSQP', 1e-6);   
            
% Optimization Funtion Cost
f_obj1 = @(x)Tele_system_E(x, h1, h2, t, t_s, L1_s, L2_s, L1_m, L2_m, qs, qm, XP);
f_obj2 = @(x)RMS_constraint(x, h1, h2, t, t_s, L1_s, L2_s, L1_m, L2_m, qs, qm, RMSE_xd, RMSE_yd, XP);
% Optimization restriccions

% Limits Gains
LB = [0.1;0.1];
UB = [30;30];

chi = fmincon(f_obj1,X,[],[],[],[],LB,UB,f_obj2,options);

save("Parameters.mat", "chi");