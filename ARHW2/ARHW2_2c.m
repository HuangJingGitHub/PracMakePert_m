%%% MAEG5402 Advanced Robotics HW2_2c Inverse Kinematics with Redundancy
%%% Vt:Time Version, achieve better computation speed
clc
clear
close all

dt = 0.001;
t = 0:0.001:5;
steps= length(t);
xd = [ 0.25*(1-cos(pi*t));
       0.25*(1-sin(pi*t))];
xd_d = [ 0.25*pi*sin(pi*t);
        -0.25*pi*cos(pi*t)];
q = zeros(3, steps);
q(:,1) = [0, pi/6, pi/4]';
xer = zeros(8,steps);
xer_norm = zeros(4,steps);
I = eye(3);
Coef = [0.5 1 5 10];

for loop = 1:4
K = Coef(loop) * diag([500,500]);
id = 2*loop - 1;
for i=1:steps
    xac = R3fk(q(:,i));
    xer(id:id+1,i) = xd(:,i) - xac;
    xer_norm(loop,i) = norm(xer(id:id+1,i));
    J = R3Jac(q);
    Jri = Wpinv(J,I);
    qdot = Jri*(xd_d(:,i)+K*xer(id:id+1,i));
    q(:,i+1) = q(:,i) + qdot*dt;
end
figure('Position',[50 50 800 550])
plot(t,xer_norm(loop,:),'color','k','LineWidth',1)
xlabel('\it{Time (s)}','Fontname','Times');
ylabel('||x_{error}||','Fontname','Times');
title(['Tracking Error with ', num2str(100*Coef(loop)),'% K'],'Fontname','Times');
end
