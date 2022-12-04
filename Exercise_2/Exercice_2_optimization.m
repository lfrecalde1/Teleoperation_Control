% Solve Optimization gains problem
% Optimization Teleoperation system

% Clean varibales
clc, clear all, close all;
% Gains slave and master

% General vector of the variables Gains No delay
X = [5.19584196883290;2.20120565239794];
X_KP = [10;8.96488557076845];
% System delay
h1 = 0.5;
h2 = 0.5;

% Final time definition
t_final = 40;

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
RMSE_xd = 0.048;
RMSE_yd = 0.050;

% Optimization parameters
options = optimset('Display','iter',...
                'TolFun', 1e-8,...
                'MaxIter', 20,...
                'Algorithm', 'active-set',...
                'FinDiffType', 'forward',...
                'RelLineSrchBnd', [],...
                'RelLineSrchBndDuration', 1,...
                'TolConSQP', 1e-8);   
            
% Optimization Funtion Cost
f_obj1 = @(x)Tele_system_E(x, h1, h2, t_final, L1_s, L2_s, L1_m, L2_m, qs, qm, X_KP);
f_obj2 = @(x)RMS_constraint(x, h1, h2, t_final, L1_s, L2_s, L1_m, L2_m, qs, qm, RMSE_xd, RMSE_yd, X_KP);
% Optimization restriccions

% Limits Gains
LB = [0.1;0.1];
UB = [50;50];

chi = fmincon(f_obj1,X,[],[],[],[],LB,UB,f_obj2,options);
