%% Code Proff the stability 
clc, clear all, close all

% Load Variables
load("Parameters.mat");

% System gains
X = [3.94014191427969;chi(1);0.896799897417479;chi(2)];

% Delay System
h_1 = 0.5;
h_2 = 0.5;

% Split System variables
Kg = 1;
Ks = X(1);
Km = X(3);
alpha_s = X(2);
alpha_m = X(4);

value_1 = -alpha_m*Kg + h_1 + (h_2/4)*(Ks^2)*(Kg^2)
value_2 = -alpha_s*(Km/Ks) + h_2 + (h_1/4)*(Km^2)*(Kg^2)
