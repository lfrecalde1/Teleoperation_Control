function u = kinematic_controller(x, xd, xdp, K1, K2, L)

% Get Jacobian Matrix
J =  get_J_matrix_control(x, L);
v = get_operator(x, xd, xdp, K1, K2);
u = inv(J)*(v);
end
