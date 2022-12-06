%% Code to compute the porcentage of the derivative gains
clc, clear all, close all;

% Values without delay
real = [2.41218473361655;3.94499729073964;1.96805547864400;1.05684211781156];

% Values delay optimizacion
load("Parameters.mat");

alpha_s_real = real(2);
alpha_m_real = real(4);

alpha_s_delay = chi(1)+0.5;
alpha_m_delay = chi(2)+0.5; 

diferencia_s = abs(alpha_s_real-alpha_s_delay);
diferencia_m = abs(alpha_m_real - alpha_m_delay);
value_1 = diferencia_s*100/alpha_s_real
value_2 = diferencia_m*100/alpha_m_real