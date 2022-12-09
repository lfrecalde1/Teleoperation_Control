function [V_final, index] = potential_field_final(h, obs)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
i = size(obs,1);
j = size(obs,2);
V = [];
for k = 1:1:j
    aux = exp(distance(h,obs(:,k)));
    V = -[V;0.6*aux];
end
minimum=min(V);
index = find(V == minimum);
V_final = V(index);
v_final = 0;
end
