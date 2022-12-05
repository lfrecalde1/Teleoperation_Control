function [c, ceq] = RMS_constraint(X, h1, h2, t_final, L1_s, L2_s, L1_m, L2_m, qs, qm, RMSE_xd, RMSE_yd)
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here

[RMSE_x_s, RMSE_y_s] = Tele_system_RMS(X, h1, h2, t_final, L1_s, L2_s, L1_m, L2_m, qs, qm);

%% Generacion de vector de estados de restricciones
c = [];
ceq = [];

%% Restriccion estabilidad
Ks = X(1);
Km = X(3);

alpha_s = X(2);
alpha_m = X(4);
Kg = 1;

%% Restriccion 1

lyapunov1 = -alpha_m*Kg + h1 + (h2/4)*(Ks^2)*(Kg^2);
lyapunov2 = -alpha_s*(Km/Ks) + h2 + (h1/4)*(Km^2)*(Kg^2);


c = [c lyapunov1 lyapunov2];
ceq = [ceq RMSE_x_s-RMSE_xd RMSE_y_s-RMSE_yd];
    
% c = [c 0 0];
% ceq = [ceq RMSE_x_s-RMSE_xd RMSE_y_s-RMSE_yd];
end