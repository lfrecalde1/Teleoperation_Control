%% Code Proff the stability 
clc, clear all, close all

% Load Variables
load("Parameters.mat");

% System gains
X = [2.41218473361655;chi(1)+0.5;1.96805547864400;chi(2)+0.5];

% Time definition
t_final = 40;
t_s = 0.01;
t = (0:t_s:t_final);

signal_h1 = 0.5 + 0.5*sin([t t(end)+t_s]);
signal_h2 = 0.5 - 0.5*sin([t t(end)+t_s]);

signal_h1 = delay_varying_time(signal_h1,t);
signal_h2 = delay_varying_time(signal_h2,t);

h1_max = max(signal_h1);
h2_max = max(signal_h2);


% Split System variables
Kg = 1;
Ks = X(1);
Km = X(3);
alpha_s = X(2);
alpha_m = X(4);

value_1 = -alpha_m*Kg + h1_max + (h2_max/4)*(Ks^2)*(Kg^2)
value_2 = -alpha_s*(Km/Ks) + h2_max + (h1_max/4)*(Km^2)*(Kg^2)
