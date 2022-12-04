function [c, ceq] = RMS_constraint(X, h1, h2, t_final, L1_s, L2_s, L1_m, L2_m, qs, qm, RMSE_xd, RMSE_yd, X_KP)
%UNTITLED13 Summary of this function goes here
%   Detailed explanation goes here

[RMSE_x_s, RMSE_y_s] = Tele_system_RMS(X, h1, h2, t_final, L1_s, L2_s, L1_m, L2_m, qs, qm, X_KP);

%% Generacion de vector de estados de restricciones
c = [];
ceq = [];


c = [c 0 0];
ceq = [ceq RMSE_x_s-RMSE_xd RMSE_y_s-RMSE_yd];
    
% c = [c 0 0];
% ceq = [ceq RMSE_x_s-RMSE_xd RMSE_y_s-RMSE_yd];
end