function drawpend2(state_s,m_1_s,m_2_s,m_0_s,l_1_s,l_2_s,state_m,m_1_m,m_2_m,m_0_m,l_1_m,l_2_m,x_m_0,obstacle, xd)
% Slave parameters
th_s = state_s(1);
th1_s = state_s(2);
% dimensions
W_s = 0.5*sqrt(m_0_s/5);  % cart width
H_s = 0.5*sqrt(m_0_s/5); % cart height
mr_s =0.05*sqrt(m_1_s);  % mass radius
mr1_s=0.05*sqrt(m_2_s);

% Master parameters
th_m = state_m(1);
th1_m = state_m(2);
% dimensions
W_m = 0.5*sqrt(m_0_m/5);  % cart width
H_m = 0.5*sqrt(m_0_m/5); % cart height
mr_m =0.05*sqrt(m_1_m);  % mass radius
mr1_m=0.05*sqrt(m_2_m);

% Obstacle definition
x_obs = obstacle(1, :);
y_obs = obstacle(2, :);
% Positions slave
pendx_s =   l_1_s*sin(th_s);
pendy_s =   -l_1_s*cos(th_s);

pendx1_s =  l_2_s*sin(th_s+th1_s) + l_1_s*sin(th_s);
pendy1_s = -l_2_s*cos(th_s+th1_s) - l_1_s*cos(th_s);

% Positions slave
pendx_m =  x_m_0(1) + l_1_m*sin(th_m);
pendy_m =  x_m_0(2) - l_1_m*cos(th_m);

pendx1_m =  x_m_0(1)+ l_2_m*sin(th_m+th1_m) + l_1_m*sin(th_m);
pendy1_m = x_m_0(2) -l_2_m*cos(th_m+th1_m) - l_1_m*cos(th_m);


% Draw slave
plot([0 pendx_s],[0 pendy_s],'-k','LineWidth',2); 
hold on

plot([pendx_s pendx1_s],[pendy_s pendy1_s],'-k','LineWidth',2);

% Draw master
plot([x_m_0(1) pendx_m],[x_m_0(2) pendy_m],'-b','LineWidth',2); 
hold on

plot([pendx_m pendx1_m],[pendy_m pendy1_m],'-b','LineWidth',2);

% Draw center amd obstacles
rectangle('Position',[0-W_s/2,0-H_s/2,W_s,H_s],'Curvature',.1,'FaceColor',[.5 0.5 1],'LineWidth',1.5); % Draw cart
plot(x_obs, y_obs,'.','LineWidth',1);
rectangle('Position',[x_m_0(1)-W_m/2,x_m_0(2)-H_m/2,W_m,H_m],'Curvature',.1,'FaceColor',[.5 0.5 1],'LineWidth',1.5); % Draw cart

% Slave 
rectangle('Position',[pendx_s-mr_s/2,pendy_s-mr_s/2,mr_s,mr_s],'Curvature',1,'FaceColor',[1 0.1 .1],'LineWidth',1.5);
rectangle('Position',[pendx1_s-mr1_s/2,pendy1_s-mr1_s/2,mr1_s,mr1_s],'Curvature',1,'FaceColor',[1 0.1 .1],'LineWidth',1.5);

% Master
rectangle('Position',[pendx_m-mr_m/2,pendy_m-mr_m/2,mr_m,mr_m],'Curvature',1,'FaceColor',[1 0.5 .1],'LineWidth',1.5);
rectangle('Position',[pendx1_m-mr1_m/2,pendy1_m-mr1_m/2,mr1_m,mr1_m],'Curvature',1,'FaceColor',[1 0.5 .1],'LineWidth',1.5);


rectangle('Position',[xd(1)-mr1_s/2,xd(2)-mr1_s/2,mr1_s,mr1_s],'Curvature',1,'FaceColor',[1 0.8 .1],'LineWidth',1.5);

axis([-1 3 -1 1]);
axis equal
grid on;
set(gcf,'Position',[100 100 1000 800])
drawnow
hold off;