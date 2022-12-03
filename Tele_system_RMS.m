function [RMSE_x_s, RMSE_y_s] = Tele_system_RMS(X, h1, h2, t_final)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Time defintion variables
t_s = 0.01;
t = (0:t_s:t_final);

g = 9.8;
% System parameters L1 slave
b1_s = 0;
m1_s = 0.8;
l1_s = 1;
Iz1_s= 0.5;

% System parameters L2 slave
b2_s = 0;
m2_s = 0.8;
l2_s = 1;
Iz2_s= 0.5;


L1_s = [b1_s , m1_s, l1_s, Iz1_s];
L2_s = [b2_s , m2_s, l2_s, Iz2_s];

% Initial conditions system slave       
q_s = zeros(4, length(t)+1);
q_s(:, 1) = [170*pi/180;...
           -35*pi/180;...
           0;...
           0];
       
% System parameters L1 slave
b1_m = 0.0;
m1_m = 0.8;
l1_m = 1;
Iz1_m= 0.5;

% System parameters L2 slave
b2_m = 0.0;
m2_m = 0.8;
l2_m = 1;
Iz2_m= 0.5;


L1_m = [b1_m , m1_m, l1_m, Iz1_m];
L2_m = [b2_m , m2_m, l2_m, Iz2_m];

% Initial conditions system master       
q_m = zeros(4, length(t)+1);
q_m(:, 1) = [90*pi/180;...
             5*pi/180;...
           0;...
           0];
       
       
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

% Desired angles of the system
qd = [90*pi/180*ones(1, length(t));...
       0*pi/180*ones(1, length(t))];
   
qdp = [0*pi/180*ones(1, length(t));...
       0*pi/180*ones(1, length(t))];
   
qdpp = [0*pi/180*ones(1, length(t));...
        0*pi/180*ones(1, length(t))];

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
wn_s = sqrt(kp_s);
kv_s = X(2);
KP_s= kp_s*eye(2);
KD_s = kv_s*eye(2);

% Control master gains
kp_m = X(3);
wn_m = sqrt(kp_m);
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
qe = zeros(2, length(t));
qep = zeros(2, length(t));

xe = zeros(2, length(t));
xep = zeros(2, length(t));


% External Force Reaction
x_enviroment_c = (-0.5:0.01:0.5);
x_enviroment_c(2, :) = 0*ones(1, length(x_enviroment_c));

% Location New Axis
r_c = [1.2;0];

% Angle new Axis
angle_c = 45*pi/180;

Rot_c = [cos(angle_c), -sin(angle_c);...
         sin(angle_c), cos(angle_c)];
     
x_enviroment_i = r_c + Rot_c*x_enviroment_c;     
% Enviroment Force
F_enviroment = zeros(2, length(t)+1);


% Potential Field
[V, index] = potential_field(x_s(:, 1), x_enviroment_i(:, :));

% Angle Between end effector and Obstacle
beta = angle_obstacle(x_s(:, 1), x_enviroment_i(:, index));

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
xdpp_i = Rot_c*xdpp_c;

% Desired Forces
F_desired = [4*ones(1, length(t)+1);...
             4*ones(1, length(t)+1)];
         
F_desired_p = [0*ones(1, length(t)+1);...
               0*ones(1, length(t)+1)];
           

% Derivative of the Force
Force_c_filter_p = zeros(2, length(t));

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

% Auxiliar Normalize 
he_s_max = abs(x_m_base_delay(:, 1) - x_s(:, 1));
he_m_max = abs(x_s_delay(:, 1) - x_m_base(:, 1));

% Control Effort
U_s = [];
U_m = [];
for k = 1:length(t)
    % Disntance Forces 
    [V(:, k), index] = potential_field(x_s_delay(:, k), x_enviroment_i(:, :));
    
    %Angle To Obstacle genral Coordinate
    beta(:, k) = angle_obstacle(x_s_delay(:, k), x_enviroment_i(:, index));
    
    %External Force
    F_enviroment(:, k) = [cos(beta(:, k));sin(beta(:, k))]*V(:, k);
    
    %External Force frame C
    F_c(:, k) = inv(Rot_c)*F_enviroment(:, k);
    
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
    
    % Control vector error articular
    qe(:, k) = qd(:, k) - q_s_delay(1:2, k);
    qep(:, k) = qdp(:, k) - q_s_delay(3:4, k);
    
    % Control vector slave master
    he_s(1, k) = (x_m_base_delay(1, k) - x_s(1, k));
    he_s(2, k) = (x_m_base_delay(2, k) - x_s(2, k));

    he_m(1, k) = (x_s_delay(1, k) - x_m_base(1, k));
    he_m(2, k) = (x_s_delay(2, k) - x_m_base(2, k));

    
    %Position Cartesian Space
    xe(:, k) = xd_i(:, k) - robot_s.get_general_position();
    xep(:, k) = xdp_i(:, k) - robot_s.get_general_velocities();
    
    % Control action
    u_cartesian_s(:, k) = control_s.get_control_tele_cartesian(q_s(:,k), x_s(:, k), xp_s(:, k), x_m_base_delay(:, k));
    u_cartesian_m(:, k) = control_m.get_control_tele_cartesian(q_m(:, k), x_m_base(:, k), xp_m(:,k),  x_s_delay(:, k));
    
    U_s = [U_s;u_cartesian_s(:, k)];
    U_m = [U_m;u_cartesian_m(:, k)];
    
    % System evolution Slave
    q_s(:, k+1) = robot_s.system_f(u_cartesian_s(:, k), Force_filter(:, k));
    x_s(:, k+1) = robot_s.get_general_position();
    xp_s(:, k+1) = robot_s.get_general_velocities();
    
    % System evolution master
    q_m(:, k+1) = robot_m.system_f(u_cartesian_m(:, k), Force_filter(:, k));
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

% Control Effort
U_s_max = max(U_s);
U_m_max = max(U_m);

U_s = U_s/U_s_max;
U_m = U_m/U_m_max;

E = U_s'*U_s + U_m'*U_m;

% for k = 1:10:length(t)
%     drawpend2(q_s_delay(:, k), m1_s, m2_s, 0.3, l1_s, l2_s, q_m_delay(:, k), m1_m, m2_m, 0.3, l1_m, l2_m, x_m_0, x_enviroment_i(:, :), xd_i(:,k));
% end
end