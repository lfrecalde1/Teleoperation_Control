function [signal_n] = signals_generator_sin(t,w,a)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%% Original signal
signal = a*sin(w*t);

signal_1 = (a*2)*sin(2*w*t);

signal_2 = (a/2)*sin(4*w*t);

signal_3 = (a/2)*sin(8*w*t);


signal_n  = signal + signal_1 + signal_2 + signal_3;
end

