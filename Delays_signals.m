%% Code to Introduce Delays in signals

%% Clean variables 
clc, clear all, close all;

%% Time defintion
t_s = 0.01;
t_final = 10;
t = (0:t_s:t_final);

% Real  Signal
signal = cos([t t(end)+t_s])';

delay = 0.5 + 0.5*sin(2*[t t(end)+t_s]);
% delay = 1*ones(1, length(t)+1);
delay = delay_varying_time(delay,t);
n_frames = round(delay/t_s);
% Delay Signal
signal_delay = delayseq(signal, delay, 1/t_s);

signal_custom_delay = zeros(1, length(t)+1);

% Initial Condition
signal_custom_delay(:, 1:n_frames(1)+1)  = signal(1,:);
signal = signal';
for k = 1:length(t)
    
    
    % Delay signal update system
   

    
    
    if k + n_frames(k+1) < (length(t)+ 1)
        signal_custom_delay(:, k+n_frames(k+1)+1:end) = signal(k + 1);
    end
    
end

figure(1)
plot(t, signal(1:length(t)), 'r');
hold on
grid on
plot(t, signal_delay(1:length(t)), 'b');
plot(t, signal_custom_delay(1:length(t)), '--m');
plot(t, delay(1:length(t)), 'b');

