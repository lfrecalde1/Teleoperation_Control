% Solve Optimization gains problem
% Optimization Teleoperation system

% Clean varibales
clc, clear all, close all;
% Gains slave and master

kp_s = 2;
kv_s = 2.5;

kp_m = 2;
kv_m = 2.5;

% General vector of the variables
X = [kp_s;kv_s;kp_m;kv_m];

% System delay
h1 = 0.0;
h2 = 0.0;

% Final time definition
t_final = 40;

% Desired RMSE
RMSE_xd = 0.1140;
RMSE_yd = 0.1603;

% Optimization parameters
options = optimset('Display','iter',...
                'TolFun', 1e-5,...
                'MaxIter', 20,...
                'Algorithm', 'active-set',...
                'FinDiffType', 'forward',...
                'RelLineSrchBnd', [],...
                'RelLineSrchBndDuration', 1,...
                'TolConSQP', 1e-5);   
            
% Optimization Funtion Cost
f_obj1 = @(x)Tele_system_E(x, h1, h2, t_final);
f_obj2 = @(x)RMS_constraint(x, h1, h2, t_final, RMSE_xd, RMSE_yd);
% Optimization restriccions

% Limits Gains
LB = [0;0;0;0];
UB = [50;50;50;50];

control = fmincon(f_obj1,X,[],[],[],[],LB,UB,f_obj2,options);

