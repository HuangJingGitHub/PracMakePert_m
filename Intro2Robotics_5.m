%%% Ideal simple simulated resolved-rate control for a planar 3-DOF, 3R robot
%%% Describtions in John Craig's book chapter5, page 173-175 (4th edition).
%%% Given desired constant Cartesian velocity, wrench of the end-effector,
%%% calculate the command angle vector during the simulation interval.
clear
close all
clc

%%% Robot configurations
DR = pi/180;
L1 = 4;
L2 = 3;
L3 = 2;

%%% Simulation configurations
Time = 5;
dt = 0.1;
t = 0:dt:Time;
Len = length(t);
x_d = [0.2 -0.3 -0.2]';
F = [1 2 3]';
J = ones(3,3);    % Jacobian matrix in base frame is:
                  % J = [ -L1s1-L2s12-L3s123 -L2s12-L3s123 -L3s123;
                  %       L1c1+L2c12+L3c123  L2c12+L3c123  L3c123;
                  %       1                  1             1]
q = zeros(3, Len);
q_d = zeros(3, Len);
torque = zeros(3, Len);
det_J = zeros(1, Len);

%%% Initial joint angles
q(1,1) = 10 * DR;
q(2,1) = 20 * DR;
q(3,1) = 30 * DR;


for loop = 1:length(t)-1
    q1 = q(1,loop);                   % theta1
    q12 = q(1,loop) + q(2,loop);      % theta1+theta2
    q123 = q12 + q(3,loop);           % theta1+theta2+theta3
    
    %%% Calculate Jacobian matrix under each pose/time step
    J(1,1) = -L1*sin(q1) - L2*sin(q12) - L3*sin(q123);
    J(1,2) = J(1,1) + L1*sin(q1);       % better efficiency than -L2*sin(ang12) - L3*sin(ang123);
    J(1,3) = -L3*sin(q123);
    J(2,1) = L1*cos(q1) + L2*cos(q12) + L3*cos(q123);
    J(2,2) = J(1,1) - L1*cos(q1);       % L2*cos(ang12) + L3*cos(ang123);
    J(2,3) = L3*cos(q123);
    det_J(loop) = abs(det(J));
    
    q_d(:,loop) = J\x_d;                % dot_ang = inv(J)*dot_x
    q(:,loop+1) = q(:,loop) + q_d(:,loop)*dt;  % Suppose the commanded joint angles are perfectly achieved
    torque(:,loop) = J'*F;              % torque = J transpose * F
end

%%% Visualize results
%%% (1) dot_theta vs time
figure('position',[0 -200 1200 800]);
plot(t(1:Len-1), q_d(1,1:Len-1),'color','k','linewidth',2);  % The last entry of dot_ang is not updated in the loop
hold on
plot(t(1:Len-1), q_d(2,1:Len-1),'color','g','linewidth',2);
hold on
plot(t(1:Len-1), q_d(3,1:Len-1),'color','b','linewidth',2);
xlabel('$Time$ $(s)$','interpreter','latex','fontsize',15);
ylabel('$Angular$ $Velocity$ $(s^{-1})$','interpreter','latex','fontsize',15);
h1=legend('$\dot{\theta_1}$',...
          '$\dot{\theta_2}$',...
          '$\dot{\theta_3}$');
set(h1,'interpreter','latex','Box','off','fontsize',18);

%%% (2) theta vs time
figure('position',[0 -200 1200 800]);
plot(t, q(1,:),'color','k','linewidth',2);
hold on
plot(t, q(2,:),'color','g','linewidth',2);
hold on
plot(t, q(3,:),'color','b','linewidth',2);
xlabel('$Time$ $(s)$','interpreter','latex','fontsize',15);
ylabel('$Joint$ $Angles$ $(rad)$','interpreter','latex','fontsize',15);
h1=legend('$\theta_1$',...
          '$\theta_2$',...
          '$\theta_3$');
set(h1,'interpreter','latex','Box','off','fontsize',18);

%%% (3) absolute value of determinant |J| vs time
figure('position',[0 -200 1200 800]);
plot(t(1:Len-1), det_J(1:Len-1),'color','k','linewidth',2);
xlabel('\fontsize{13}\fontname{Times}Time (s)');
ylabel('\fontsize{13}\fontname{Times}|J|');


%%% Use Croke's toolbox to animate robot's motion
L(1) = Link([0, 0, 0, 0]);    % Start from link0(base).
L(2) = Link([0, 0, L1, 0]);     
L(3) = Link([0, 0, L2, 0]);
L(4) = Link([0, 0, L3, 0]);  
robot3R = SerialLink(L, 'name', '3R_PlanarRobot');

figure('Position',[0 -200 1500 800]);
 for loop = 1:length(t)
     robot3R.plot([0 q(:,loop)']);    % Input is a column vector
     view(0,55);
     pause(0.2);
 end
