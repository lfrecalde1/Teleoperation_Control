function [beta] = angle_obstacle(x, x_env)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% Position of the end efector
x_real = x(1, 1);
y_real = x(2, 1);

% Position of the Obstacle
x_obs = x_env(1, 1);
y_obs = x_env(2, 1);

beta = atan2((y_obs-y_real),(x_obs-x_real));
end

