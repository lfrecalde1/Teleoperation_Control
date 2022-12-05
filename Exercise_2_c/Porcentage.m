%% Code to compute the porcentage of the derivative gains
clc, clear all, close all;

% Values without delay
real = [3.94014191427969;3.43669151142530;0.896799897417479;0.477142268540486];

% Values delay optimizacion
load("Parameters.mat");

alpha_s_real = real(2);
alpha_m_real = real(4);

alpha_s_delay = chi(1);
alpha_m_delay = chi(2); 

value_1 = alpha_s_delay*100/alpha_s_real
value_2 = alpha_m_delay*100/alpha_m_real