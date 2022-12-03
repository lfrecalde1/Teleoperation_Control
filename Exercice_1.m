% Optimization Teleoperation system

% Clean varibales
clc, clear all, close all;
% Gains slave and master



% General vector of the variables Gains No delay
X = [5.89907241495536;2.88537941103166;5.08265637272528;1.79620808067804];

% General vector of the variables gains Delay 
%X = [48.7474744760916;42.4307447730334;7.33940221283621;3.86267531763958];

% System delay
h1 = 0.0;
h2 = 0.0;

% Final time definition
t_final = 40;

[RMSE_x, RMSE_y] = Tele_system_simu(X, h1, h2, t_final)
