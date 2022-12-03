%% Simulation fo the mobile Robot %%

% Clear variables
clc, clear all, close all;

% Time definition variables
t_s = 0.01;
t_final = 30;
t = (0:t_s:t_final);

% System Geometric Parameters
a = 0.2;

% Genearl vector parameters
L1 = [a];

% Initial conditions of the system
x_i = 0;
y_i = 0;
theta = 0*pi/180;

% Initial conditions of the system general vector
q = zeros(3, length(t)+1);

q(:, 1) = [x_i + a*cos(theta);...
           y_i + a*sin(theta);...
           theta];
       
x(:, 1) = [x_i + a*cos(theta);...
           y_i + a*sin(theta);...
           theta];

% Dynamic parameters
chi = [0.3037;0.2768;-0.0004018;0.9835;0.003818;1.0725];

% Robot mobile definitition
mobile_1 = mobile_robot(L1, chi, q(:,1), t_s);

% Desired Trajectory
[xd,yd,zd,psid,xdp,ydp,zdp,psidp] = Trajectory(t, t_s, 3);   
% Desired Trajectory
qd = [xd;...
      yd];
  
qdp = [xdp;...
       ydp];
   
% Control Signal
u = [sin(0.5*t);...
     cos(0.5*t)];
     
% Control gains
K1 = 1*eye(2);
K2 = 1*eye(2);

% Delaydefinition
delay = 0.05;
n_frames = (delay/t_s);

% Delay Auxiliar signal
q_delay = zeros(3, length(t)+1);
q_delay(:, n_frames + 1) = x(:, 1);

% Loop Simulation
for k = 1:length(t)
    % Control Error
    qe(:, k) = qd(:, k) - q_delay(1:2,k);
    
    % Control Law section
    u(:, k) = kinematic_controller(q_delay(:, k), qd(:, k), qdp(:, k), K1, K2, L1);

    % System evolution
    q(:, k+1) = mobile_1.system_f(u(:, k));
    
    x(:, k+1) = system_f(x(:, k), u(:, k), t_s, L1);
    
    % Delay signal update system
    if k + n_frames < (length(t)+ 1)
        q_delay(:, k + n_frames + 1) = x(:, k + 1); 
    end
end

largo = 0.4;
ancho = 0.3;
SIMULACION(a,largo,ancho,q(1,:),q(2,:),qd(1, :),qd(2, :),q(3, :),t_s);
% Save Data
save("Mobile_Kinematics.mat", "t", "q", "u");

% Delay Signals
figure(2)
plot(t, q(1,1:length(t)));
hold on
grid on
plot(t, q_delay(1,1:length(t)), "--");

figure(3)
plot(t, qe(:,1:length(t)));
hold on
grid on

