function xp = f_model(x, u, L)
%Gets funtion dot of the system

% Get Jacobian matrix
J = J_matrix(x, L);

% Get system
xp = J*u;
end
