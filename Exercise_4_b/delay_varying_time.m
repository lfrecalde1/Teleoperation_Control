function [signal_round] = delay_varying_time(signal,t)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
i = size(signal,1);
j = size(signal,2);

signal_round = zeros(i,j);

signal_round(:, 1) = signal(:, 1);

for k = 1:length(t)
    
    
    signal_round(:, k+1) = round(signal(:, k+1),2);
end
end

