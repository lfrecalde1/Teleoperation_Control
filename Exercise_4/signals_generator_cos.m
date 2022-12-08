function [signal_n] = signals_generator_cos(t,w,a)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%% Original signal
signal = a*cos(w*t);

signal_1 = (a*2)*cos(2*w*t);

signal_2 = (a/2)*cos(4*w*t);

signal_3 = (a/2)*cos(8*w*t);


signal_n  = signal + signal_1;
end
