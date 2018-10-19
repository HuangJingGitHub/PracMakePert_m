%%% MAEG5402 Advanced Robotics HW2_2a Inverse Kinematics with Redundancy
%%% Vt:Time Version, achieve better computation speed
clc
clear
close all

dt = 0.001;
t = 0:0.001:2.4;
steps= length(t);
xd = [ 0.25*(1-cos(pi*t));
       0.25*(1-sin(pi*t))];
xd_d = [ 0.25*pi*sin(pi*t);
        -0.25*pi*cos(pi*t)];
K = diag([500,500]);
q = zeros(3, steps);
q(:,1) = [0, pi/6, pi/4]';
xac = zeros(2, steps);
PosJoit = zeros(4, steps);
I = eye(3);

for i=1:steps
    xac(:,i) = R3fk(q(:,i));
    PosJoit(:,i) = R3posjoit(q(:,i));
    xer = xd(:,i) - xac(:,i);
    J = R3Jac(q);
%     [U,S,V] = svd(J);
%     Jri = V*S'/(S*S')*U';
    Jri = Wpinv(J,I);
    qdot = Jri*(xd_d(:,i)+K*xer);
    q(:,i+1) = q(:,i) + qdot*dt;
end

fig1 = figure('position',[50 80 850 500]);
link1 = line([0 PosJoit(1,1)],[0 PosJoit(2,1)],...
             'Color','k','Marker','.','MarkerSize',10,'LineWidth',1.5);
link2 = line([PosJoit(1,1) PosJoit(3,1)],[PosJoit(2,1) PosJoit(4,1)],...
             'Color','k','Marker','.','MarkerSize',10,'LineWidth',1.5);
link3 = line([PosJoit(3,1) xac(1,1)],[PosJoit(4,1) xac(2,1)],...
             'Color','k','Marker','.','MarkerSize',10,'LineWidth',1.5);
axis([-1 3 -1.2 1.2])
xlabel('X (m)','fontname','Times');
ylabel('Y (m)','fontname','Times')
hold on

for i=1:5:steps         % The points are so dense, so pick a point every 5 data.
    plot(xac(1,i),xac(2,i),'Marker','.','Color','r','MarkerSize',5)
    set(link1,'xdata',[0 PosJoit(1,i)],'ydata',[0 PosJoit(2,i)]);
    set(link2,'xdata',[PosJoit(1,i) PosJoit(3,i)],'ydata',[PosJoit(2,i) PosJoit(4,i)]);
    set(link3,'xdata',[PosJoit(3,i) xac(1,i)],'ydata',[PosJoit(4,i) xac(2,i)]);
    %axis equal
    hold on
    pause(2*dt);
end