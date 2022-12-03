function v = get_operator(x, xd, xdp, K1, K2)

% Auxiliar signal
xe = xd - x(1:2);
v =  (xdp + K2*tanh(inv(K2)*K1*xe));
end