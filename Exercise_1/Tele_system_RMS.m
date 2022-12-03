function [RMSE_x_s, RMSE_y_s] = Tele_system_RMS(X, h1, h2, t_final, L1_s, L2_s, L1_m, L2_m, qs, qm)

%UNTITLED Summary of this function goes here
% Time defintion variables
t_s = 0.01;
t = (0:t_s:t_final);
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

% Auxiliar rand

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
x_m_0 = [3; 0];
x_m(:, 1) = x_m_0 + robot_m.get_general_position();
xp_m(:, 1) = robot_m.get_general_velocities();

% Cartesian space master respect B
x_m_base(:, 1) = x_m_0 + robot_m.get_general_position()- x_m_0;


% Desired Position cartesian Space
xd_c = [0.3*sin(0.1*t);...
      0.1*ones(1, length(t))];
  
xdp_c = [(0.3*0.1)*cos(0.1*t);...
       0*ones(1, length(t))];

xdpp_c = [-(0.3*0.1*0.1)*sin(0.1*t);...
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
x_enviroment_c = (-0.5:0.01:0.5);
x_enviroment_c(2, :) = 0*ones(1, length(x_enviroment_c));

% Location New Axis Enviroment
r_c = [1.2;0];

% Angle new Axis
angle_c = 45*pi/180;

Rot_c = [cos(angle_c), -sin(angle_c);...
         sin(angle_c), cos(angle_c)];
     
x_enviroment_i = r_c + Rot_c*x_enviroment_c;     

% Deisred Trayectory I frame
xd_i = r_c + Rot_c*xd_c;
xdp_i = Rot_c*xdp_c;

% Delay s time definition
delay_s = h2;
n_frames_s = delay_s/t_s;

% Delay signals slave
q_s_delay = zeros(4, length(t)+1);
x_s_delay = zeros(2, length(t)+1);

% Initial Condition
q_s_delay(:, 1:n_frames_s(1) + 1)  = q_s(:,1)*ones(1,n_frames_s + 1);
x_s_delay(:, 1:n_frames_s(1) + 1) = x_s(:, 1)*ones(1,n_frames_s + 1);

% Delay m time definition
delay_m = h1;
n_frames_m = delay_m/t_s;

% Delay signals slave
q_m_delay = zeros(4, length(t)+1);
x_m_delay = zeros(2, length(t)+1);
x_m_base_delay = zeros(2, length(t)+1);

% Initial Condition
q_m_delay(:, 1:n_frames_m(1) + 1)  = q_m(:,1)*ones(1,n_frames_m + 1);
x_m_delay(:, 1:n_frames_m(1) + 1) = x_m(:, 1)*ones(1,n_frames_m + 1);
x_m_base_delay(:, 1:n_frames_m(1) + 1) = x_m_base(:, 1)*ones(1,n_frames_m + 1);


% Control Effort
U_s = [];
U_m = [];

% Control Error
He_s = [];
He_m = [];

% Velocity Effort
Qxp_m = [];
Qxp_s = [];

for k = 1:length(t)
    
    
    % Control vector slave master
    he_s(1, k) = (x_m_base_delay(1, k) - x_s(1, k));
    he_s(2, k) = (x_m_base_delay(2, k) - x_s(2, k));
    
    
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
    
    % Control action
    u_cartesian_s(:, k) = control_s.get_control_tele_cartesian(q_s(:,k), x_s(:, k), xp_s(:, k), x_m_base_delay(:, k));
    u_cartesian_m(:, k) = control_m.get_control_tele_cartesian(q_m(:, k), x_m_base(:, k), xp_m(:,k),  x_s_delay(:, k));
    
    U_s = [U_s;u_cartesian_s(:, k)];
    U_m = [U_m;u_cartesian_m(:, k)];
    
    % System evolution Slave
    q_s(:, k+1) = robot_s.system_f(u_cartesian_s(:, k), [0; 0]);
    x_s(:, k+1) = robot_s.get_general_position();
    xp_s(:, k+1) = robot_s.get_general_velocities();
    
    % System evolution master
    q_m(:, k+1) = robot_m.system_f(u_cartesian_m(:, k), [0; 0]);
    x_m(:, k+1) = x_m_0 + robot_m.get_general_position();
    x_m_base(:, k+1) = x_m(:, k+1) - x_m_0;
    xp_m(:, k+1) = robot_m.get_general_velocities();
    
    % Delay system 
     if k + n_frames_s < (length(t)+ 1)
        q_s_delay(:, k + n_frames_s + 1) = q_s(:, k + 1); 
        x_s_delay(:, k + n_frames_s + 1) = x_s(:, k + 1); 
     end
     
     if k + n_frames_m < (length(t)+ 1)
         q_m_delay(:, k + n_frames_m + 1) = q_m(:, k + 1);
         x_m_delay(:, k + n_frames_m + 1) = x_m(:, k + 1);
         x_m_base_delay(:, k + n_frames_m + 1) = x_m_base(:, k+1);
     end
     

end

RMSE_x_s = sqrt(mean((he_s(1,:)).^2));
RMSE_y_s = sqrt(mean((he_s(2,:)).^2));

E = 0.1*(U_s'*U_s) + 0.1*(U_m'*U_m) + (He_m'*He_m) + (He_s'*He_s) + 1*(Qxp_m'*Qxp_m) + 1*(Qxp_s'*Qxp_s);

end