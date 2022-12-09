function [q_s, q_m, q_s_delay, q_m_delay, x_s, x_m_base, x_s_delay, x_m_base_delay, xp_s, xp_m, xp_s_delay, xp_m_delay, he_m, he_s, RMSE_x_s, RMSE_y_s, u_cartesian_s, u_cartesian_m, x_m_0, e, F_h, F_enviroment, xd_i, x_enviroment_i] = Tele_system_simu(X, h1, h2, t, t_s, L1_s, L2_s, L1_m, L2_m, qs, qm, kp_h, kp_e)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Time defintion variables
g = 9.8;

% Constant values Master
% System parameters L1 master
b1_m = L1_m(1);
m1_m = L1_m(2);
l1_m = L1_m(3);
Iz1_m= L1_m(4);

% System parameters L2 master
b2_m = L2_m(1);
m2_m = L2_m(2);
l2_m = L2_m(3);
Iz2_m= L2_m(4);

% Constant values Slave
% System parameters L1 Slave
b1_s = L1_s(1);
m1_s = L1_s(2);
l1_s = L1_s(3);
Iz1_s= L1_s(4);

% System parameters L2 Slave
b2_s = L2_s(1);
m2_s = L2_s(2);
l2_s = L2_s(3);
Iz2_s= L2_s(4);

% Initial conditions system slave       
q_s = zeros(4, length(t)+1);
q_s(:, 1) = qs;
       
% Initial conditions system master       
q_m = zeros(4, length(t)+1);
q_m(:, 1) = qm;
       
       
% Constant defintion
constans = [g, t_s];

% Robot definition slave
robot_s = manipulator_system(L1_s, L2_s, constans, q_s(:,1));

% Robot definition Master
robot_m = manipulator_system(L1_m, L2_m, constans, q_m(:,1));

% Inital Position Carterian Space slave
x_s = zeros(2, length(t)+1);
xp_s = zeros(2, length(t)+1);
x_s(:, 1) = robot_s.get_general_position();
xp_s(:, 1) = robot_s.get_general_velocities();

% Inital Position Carterian Space master
x_m = zeros(2, length(t)+1);
x_m_base = zeros(2, length(t)+1);
xp_m = zeros(2, length(t)+1);
x_m_0 = [2; 0];
x_m(:, 1) = x_m_0 + robot_m.get_general_position();
xp_m(:, 1) = robot_m.get_general_velocities();

% Cartesian space master respect B
x_m_base(:, 1) = x_m_0 + robot_m.get_general_position()- x_m_0;


% Desired Position cartesian Space
xd_c = [0.0*ones(1, length(t));...
        0.0*ones(1, length(t))];
  
xdp_c = [0.0*ones(1, length(t));...
       0*ones(1, length(t))];

xdpp_c = [0.0*ones(1, length(t));...
        0*ones(1, length(t))];

% Control gains 
kp = 5;
wn = sqrt(kp);
kv = 8*1*wn;

ki = 5;
kp_force = 2;
wn_force = sqrt(kp_force);
kv_force = 20*1*wn_force;

% Learning variable
max_value = (((kv^2)/2)-2)/(sqrt((kv^2)/(4)-1));

gamma = 0.1;

% Learning system
% Transfer funtion definitions
P = tf([1 (kv-gamma) kp-gamma], [0 0 1]);
A = tf([1], [1 kv kp]);
S = tf([0 0 1], [0 1 0]);

% Filter force design
kp_f = 1;
wn_f = sqrt(kp_f);
kv_f = 1.2*1*wn_f;

Filter = tf([1], [1 kv_f kp_f]);
Filter_d = c2d(Filter, t_s);
[num1d_filter, den1d_filter] = tfdata(Filter_d,'v');

% Filter coeficients
A_filter = num1d_filter(2:end);
B_filter = den1d_filter(2:end);

% Operator contraction
aux = S*P*A;

% Operator Function
aux_d = c2d(aux, t_s);
[num1d, den1d] = tfdata(aux_d,'v');


% PD control Gains
K1 = kp*eye(2);
K2 = kv*eye(2);
Km = ki*eye(2);

% Cntrol force gains
KP_force = kp_force*eye(2);
KD_force = kv_force*eye(2);

% Control slave gains
kp_s = X(1);
kv_s = X(2);
KP_s= kp_s*eye(2);
KD_s = kv_s*eye(2);

% Control master gains
kp_m = X(3);
kv_m = X(4);
KP_m= kp_m*eye(2);
KD_m = kv_m*eye(2);


% Controller definition slave
control_s = controller(K1, K2, Km, KP_force, KD_force, KP_s, KD_s, num1d, den1d, robot_s);

u_cartesian_s = zeros(2, length(t));

% Controller definition master
control_m = controller(K1, K2, Km, KP_force, KD_force, KP_m, KD_m, num1d, den1d, robot_m);

u_cartesian_m = zeros(2, length(t));
% Control vector error definition
xe = zeros(2, length(t));
xep = zeros(2, length(t));


% External Force Reaction
x_enviroment_c = (-0.1:0.01:0.1);
x_enviroment_c(2, :, 1) = 0*ones(1, length(x_enviroment_c));

% Location New Axis Enviroment
r_c = [0.5;0.0];

% Angle new Axis
angle_c = 45*pi/180;

Rot_c = [cos(angle_c), -sin(angle_c);...
         sin(angle_c), cos(angle_c)];
     
x_enviroment_i = r_c + Rot_c*x_enviroment_c;     
% x_enviroment_i = [0.2 + 0.1*sin(0.01*t);...
%                   0*ones(1, length(t))];
% Enviroment Force
F_enviroment = zeros(2, length(t)+1);


% Potential Field
[V, index] = potential_field_final(x_s(:, 1), x_enviroment_i(:, :));

% Angle Between end effector and Obstacle
beta = angle_obstacle(x_s(:, 1), x_enviroment_i(:, index));

% Force In I
F_enviroment(:, 1) = [cos(beta);sin(beta)]*V;

% Forces Frame C
F_c(:,1) = inv(Rot_c)*F_enviroment(:,1);

% Force Filter
F_x_memory = zeros(length(den1d_filter(2:end)),1);
F_y_memory = zeros(length(den1d_filter(2:end)),1);

F_x_filter =  zeros(length(num1d_filter(2:end)),1);
F_y_filter =  zeros(length(num1d_filter(2:end)),1);
% Deisred Trayectory I frame
xd_i = r_c + Rot_c*xd_c;
xdp_i = Rot_c*xdp_c;

xd_i = [0.5 + 0.2*sin(0.05*t);...
        0.0*ones(1, length(t))];

% Delay s time definition
delay_s = h2;
n_frames_s = round(delay_s/t_s);

% Delay signals slave
q_s_delay = zeros(4, length(t)+1);
x_s_delay = zeros(2, length(t)+1);
xp_s_delay = zeros(2, length(t)+1);
F_e_delay = zeros(2, length(t)+1);

% Initial Condition
q_s_delay(:, 1:n_frames_s(1) + 1)  = q_s(:,1)*ones(1,n_frames_s(1) + 1);
x_s_delay(:, 1:n_frames_s(1) + 1) = x_s(:, 1)*ones(1,n_frames_s(1) + 1);
xp_s_delay(:, 1:n_frames_s(1) + 1) = xp_s(:, 1)*ones(1,n_frames_s(1) + 1);
F_e_delay(:, 1:n_frames_s(1) + 1) = F_enviroment(:, 1)*ones(1,n_frames_s(1) + 1);
% Delay m time definition
delay_m = h1;
n_frames_m = round(delay_m/t_s);

% Delay signals slave
q_m_delay = zeros(4, length(t)+1);
x_m_delay = zeros(2, length(t)+1);
xp_m_delay = zeros(2, length(t)+1);
x_m_base_delay = zeros(2, length(t)+1);
F_h_delay = zeros(2, length(t)+1);

% Initial Condition
q_m_delay(:, 1:n_frames_m(1) + 1)  = q_m(:,1)*ones(1,n_frames_m(1) + 1);
x_m_delay(:, 1:n_frames_m(1) + 1) = x_m(:, 1)*ones(1,n_frames_m(1) + 1);
xp_m_delay(:, 1:n_frames_m(1) + 1) = xp_m(:, 1)*ones(1,n_frames_m(1) + 1);
x_m_base_delay(:, 1:n_frames_m(1) + 1) = x_m_base(:, 1)*ones(1,n_frames_m(1) + 1);
F_h_delay(:, 1:n_frames_m(1) + 1) = [0;0]*ones(1,n_frames_m(1) + 1);

% Control Effort
U_s = [];
U_m = [];

% Control Error
He_s = [];
He_m = [];

% Velocity Effort
Qxp_m = [];
Qxp_s = [];

aux_frames = ones(1, length(t)+1);

% Operator Gains
wn_h = sqrt(kp_h);
kv_h = 3*1*wn_h

KP_h = kp_h*eye(2);
KD_h = kv_h*eye(2);

% Enviroments Gains
Ke = kp_e*eye(2);
Ke(2,2) = 0;
alpha_e = 0.1*eye(2);

% enviroment
x_enviroment = [0.5*ones(1, length(t));...
                0*ones(1, length(t))];

for k = 1:length(t)
    [V(:, k), index] = potential_field_final(x_s(:, k), x_enviroment_i(:, :));
    
    %Angle To Obstacle genral Coordinate
    beta(:, k) = angle_obstacle(x_s(:, k), x_enviroment_i(:, index));
    
    %External Force
    F_enviroment(:, k) = [cos(beta(:, k));sin(beta(:, k))]*V(:, k);
    
%     if x_s(1, k) > x_enviroment(1, k)
%         F_enviroment(:, k) = Ke*(x_enviroment(:, k)-x_s(:, k)) - alpha_e*xp_s(:, k);
%     else
%         F_enviroment(:, k) = [0;0];
%     end
    
    
    % Filter Force
    [F_x_filter_k, F_x_filter, F_x_memory] = filter(F_enviroment(1, k), F_x_filter, F_x_memory, A_filter, B_filter);
    [F_y_filter_k, F_y_filter, F_y_memory] = filter(F_enviroment(2, k), F_y_filter, F_y_memory, A_filter, B_filter);
    
    % Force I frame filter
    Force_filter(:, k) = [F_x_filter_k;...
        F_y_filter_k];
    
    % Force C frame Filter
    Force_c_filter(:, k) = inv(Rot_c)*Force_filter(:, k);
    
    % Derivative Force
    if k > 1
        Force_c_filter_p(:, k) = (Force_c_filter(:, k) - Force_c_filter(:, k-1))/t_s;
    end
    
    % Control vector slave master
    he_s(1, k) = (x_m_base_delay(1, k) - x_s(1, k));
    he_s(2, k) = (x_m_base_delay(2, k) - x_s(2, k));
    
    he(:, k) = x_m_base(:, k) -x_s(:, k);
    
    e(1, k) = norm(he(:, k));
    
    He_s = [He_s;he_s(:,k)];

    he_m(1, k) = (x_s_delay(1, k) - x_m_base(1, k));
    he_m(2, k) = (x_s_delay(2, k) - x_m_base(2, k));

    He_m = [He_m;he_m(:,k)];
    
    % General vector Velocities
    Qxp_m = [Qxp_m;xp_m(:,k)];
    Qxp_s = [Qxp_s;xp_s(:,k)];
    
    %Position Cartesian Space
    xe(:, k) = xd_i(:, k) - robot_s.get_general_position();
    xep(:, k) = xdp_i(:, k) - robot_s.get_general_velocities();
    
    % Operator Law
    F_h(:, k) = KP_h*(xd_i(:,k) - x_s_delay(:, k)) - KD_h*xp_m(:, k);
    
    % Control action
    u_cartesian_s(:, k) = control_s.get_control_tele_cartesian_force_slave(q_s(:,k), x_s(:, k), xp_s(:, k), x_m_base_delay(:, k), F_h(:,k), 1);
    u_cartesian_m(:, k) = control_m.get_control_tele_cartesian_force_master(q_m(:, k), x_m_base(:, k), xp_m(:,k),  x_s_delay(:, k),F_enviroment(:, k), 1);
    
    U_s = [U_s;u_cartesian_s(:, k)];
    U_m = [U_m;u_cartesian_m(:, k)];
    
    
    % System evolution Slave
    q_s(:, k+1) = robot_s.system_f(u_cartesian_s(:, k), -F_enviroment(:, k));
    x_s(:, k+1) = robot_s.get_general_position();
    xp_s(:, k+1) = robot_s.get_general_velocities();
    
    % System evolution master
    q_m(:, k+1) = robot_m.system_f(u_cartesian_m(:, k), -F_h(:, k));
    x_m(:, k+1) = x_m_0 + robot_m.get_general_position();
    x_m_base(:, k+1) = x_m(:, k+1) - x_m_0;
    xp_m(:, k+1) = robot_m.get_general_velocities();
    
    % Delay system 
     if k + n_frames_s(k+1) < (length(t)+ 1)
        q_s_delay(:, k+n_frames_s(k+1)+1:end) = q_s(:, k + 1)*aux_frames(1, k+n_frames_s(k+1)+1:end); 
        x_s_delay(:, k+n_frames_s(k+1)+1:end) = x_s(:, k + 1)*aux_frames(1, k+n_frames_s(k+1)+1:end);
        xp_s_delay(:, k+n_frames_s(k+1)+1:end) = xp_s(:, k+1)*aux_frames(1, k+n_frames_s(k+1)+1:end);
        F_e_delay(:, k+n_frames_s(k+1)+1:end) = F_enviroment(:, k+1)*aux_frames(1, k+n_frames_s(k+1)+1:end);
     end
     
     if k + n_frames_m(k+1) < (length(t)+ 1)
         q_m_delay(:, k+n_frames_m(k+1)+1:end) = q_m(:, k + 1)*aux_frames(1, k+n_frames_m(k+1)+1:end);
         x_m_delay(:, k+n_frames_m(k+1)+1:end) = x_m(:, k + 1)*aux_frames(1, k+n_frames_m(k+1)+1:end);
         x_m_base_delay(:, k+n_frames_m(k+1)+1:end) = x_m_base(:, k+1)*aux_frames(1, k+n_frames_m(k+1)+1:end);
         xp_m_delay(:, k+n_frames_m(k+1)+1:end) = xp_m(:, k+1)*aux_frames(1, k+n_frames_m(k+1)+1:end);
         F_h_delay(:, k+n_frames_m(k+1)+1:end) = F_h(:, k)*aux_frames(1, k+n_frames_m(k+1)+1:end);

     end
     

end

RMSE_x_s = sqrt(mean((he(1,:)).^2));
RMSE_y_s = sqrt(mean((he(2,:)).^2));

E = 0.1*(U_s'*U_s) + 0.1*(U_m'*U_m) + (He_m'*He_m) + (He_s'*He_s) + 0.7*(Qxp_m'*Qxp_m) + 0.7*(Qxp_s'*Qxp_s);

for k = 1:10:length(t)
    drawpend2(q_s_delay(:, k), m1_s, m2_s, 0.3, l1_s, l2_s, q_m_delay(:, k), m1_m, m2_m, 0.3, l1_m, l2_m, x_m_0, x_enviroment_i(:, :), xd_i(:,k));
end
end
