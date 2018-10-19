%%% MAEG5402 Advanced Robotics HW2_2 Inverse Kinematics with Redundancy
clc
clear
close all

dt = 0.001;
t = 0:0.001:2.5;
steps= length(t);
xd = [ 0.25*(1-cos(pi*t));
       0.25*(1-sin(pi*t))];
xd_d = [ 0.25*pi*sin(pi*t);
        -0.25*pi*cos(pi*t)];
K = diag([500,500]);
q = zeros(3, steps);
q(:,1) = [0, pi/6, pi/4]';
%xac = zeros(2, steps)

PosJoit = R3posjoit(q(:,1));
xac = R3fk(q(:,1));

fig1 = figure('position',[50 80 850 500]);
link1 = line([0 PosJoit(1)],[0 PosJoit(2)],...
             'Color','k','Marker','.','MarkerSize',10,'LineWidth',1.5);
link2 = line([PosJoit(1) PosJoit(3)],[PosJoit(2) PosJoit(4)],...
             'Color','k','Marker','.','MarkerSize',10,'LineWidth',1.5);
link3 = line([PosJoit(3) xac(1,1)],[PosJoit(4) xac(2,1)],...
             'Color','k','Marker','.','MarkerSize',10,'LineWidth',1.5);
axis([-1 3 -1.2 1.2])
%axis equal
hold on

for i=2:steps
    xac = R3fk(q(:,i));
    PosJoit = R3posjoit(q(:,i));
    xer = xd(:,i) - xac;
    J = R3Jac(q);
    [U,S,V] = svd(J);
    Jri = V*S'/(S*S')*U';
    qdot = Jri*(xd_d(:,i)+K*xer);
    q(:,i+1) = q(:,i) + qdot*dt;
    
    figure(fig1)
    axis([-1 3 -1.2 1.2])
    plot(xac(1),xac(2),'Marker','.','Color','r','MarkerSize',5)
    set(link1,'xdata',[0 PosJoit(1)],'ydata',[0 PosJoit(2)]);
    set(link2,'xdata',[PosJoit(1) PosJoit(3)],'ydata',[PosJoit(2) PosJoit(4)]);
    set(link3,'xdata',[PosJoit(3) xac(1)],'ydata',[PosJoit(4) xac(2)]);
    %axis equal
    hold on
    %pause(dt);
end

% figure
% plot(xac(1,:),xac(2,:));
% axis equal