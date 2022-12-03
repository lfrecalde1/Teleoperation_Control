function drawpend2(state,m_1,m_2,m_0,l_1,l_2, obstacle, xd)
th = state(1);
th1=state(2);
% dimensions
W = 1*sqrt(m_0/5);  % cart width
H = 1*sqrt(m_0/5); % cart height
mr =0.2*sqrt(m_1);  % mass radius
mr1=0.2*sqrt(m_2);

x_obs = obstacle(1, :);
y_obs = obstacle(2, :);
% positions
pendx =   l_1*sin(th);
pendy =   -l_1*cos(th);

pendx1 =  l_2*sin(th+th1) + l_1*sin(th);
pendy1 = -l_2*cos(th+th1) - l_1*cos(th);


plot([0 pendx],[0 pendy],'-k','LineWidth',2); 
hold on

plot([pendx pendx1],[pendy pendy1],'-k','LineWidth',2);

rectangle('Position',[0-W/2,0-H/2,W,H],'Curvature',.1,'FaceColor',[.5 0.5 1],'LineWidth',1.5); % Draw cart
rectangle('Position',[x_obs,y_obs-2*H,W,4*H],'Curvature',.1,'FaceColor',[.5 0.5 1],'LineWidth',1.5); % Draw cart


rectangle('Position',[pendx-mr/2,pendy-mr/2,mr,mr],'Curvature',1,'FaceColor',[1 0.1 .1],'LineWidth',1.5);
rectangle('Position',[pendx1-mr1/2,pendy1-mr1/2,mr1,mr1],'Curvature',1,'FaceColor',[1 0.1 .1],'LineWidth',1.5);
rectangle('Position',[xd(1)-mr1/2,xd(2)-mr1/2,mr1,mr1],'Curvature',1,'FaceColor',[1 0.5 .1],'LineWidth',1.5);

axis([-5 5 -1 1]);
axis equal
grid on;
set(gcf,'Position',[100 100 1000 800])
drawnow, hold off