clear
close all
clc

DR = pi/180;
L1 = 4;
L2 = 3;
L3 = 2;
Time = 5;
step = 0.1;
t = 0:step:Time;
ang = zeros(3, length(t));
dot_ang = zeros(3, length(t));
torque = zeros(3, length(t));
ang(1,1) = 10 * DR;
ang(2,1) = 20 * DR;
ang(3,1) = 30 * DR;
dot_x = [0.2 -0.3 -0.2]';
F = [1 2 3]';
J = ones(3,3);

for loop = 1:length(t)-1
    ang1 = ang(1,loop);
    ang12 = ang(1,loop) + ang(2,loop);
    ang123 = ang12 + ang(3,loop);
    J(1,1) = -L1*sin(ang1) - L2*sin(ang12) - L3*sin(ang123);
    J(1,2) = J(1,1) + L1*sin(ang1);       % -L2*sin(ang12) - L3*sin(ang123);
    J(1,3) = -L3*sin(ang123);
    J(2,1) = L1*cos(ang1) + L2*cos(ang12) + L3*cos(ang123);
    J(2,2) = J(1,1) - L1*cos(ang1);       % L2*cos(ang12) + L3*cos(ang123);
    J(2,3) = L3*cos(ang123);
    dot_ang(:,loop) = J\dot_x;
    ang(:,loop+1) = ang(:,loop) + dot_ang(:,loop)*step;
    torque(:,loop) = J'*F;
end

L(1) = Link([0, 0, 0, 0]);    % Start from link0(base).      
L(2) = Link([0, 0, L1, 0]);     
L(3) = Link([0, 0, L2, 0]);
L(4) = Link([0, 0, L3, 0]);  
robot3R = SerialLink(L, 'name', '3R_PlanarRobot');

figure('Position',[0 -300 1800 900]);
for loop = 1:length(t)
    robot3R.plot([0 ang(:,loop)']);
    view(0,50);
    pause(0.2);
end
    
