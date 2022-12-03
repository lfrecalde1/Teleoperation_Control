function J = get_J_matrix_control(x, L)
%Gets the Jacobian Matrix
%   Split values of the states

q_x = x(1);
q_y = x(2);
theta = x(3);

a = L(1);
% get Jacobian Matrix of the system

J_11 = cos(theta);
J_12 = -a*sin(theta);

J_21 = sin(theta);
J_22 = a*cos(theta);

J_31 = 0;
J_32 = 1;

J = [J_11, J_12;...
    J_21, J_22];
end


