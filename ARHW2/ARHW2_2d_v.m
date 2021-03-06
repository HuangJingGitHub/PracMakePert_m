%%% MAEG5402 Advanced Robotics HW2_2d_v. Null space to maximize
%%% manipulability
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
q(:,1) = [3*pi/4, -pi/2, pi/2]';    % Initial position of robot.
xac = zeros(2, steps);
PosJoit = zeros(4, steps);
I = eye(3);
pra = linspace(0,2*pi,100);

for i=1:steps
    xac(:,i) = R3fk(q(:,i));
    PosJoit(:,i) = R3posjoit(q(:,i));
    xer = xd(:,i) - xac(:,i);
    J = R3Jac(q(:,i));
    Jri = Wpinv(J,I);
    qdntemp = R3MaxManipty(1,q(:,i));
    qdot = Jri*(xd_d(:,i)+K*xer) + (I-Jri*J)*qdntemp;
    q(:,i+1) = q(:,i) + qdot*dt;
end

fig1 = figure('position',[50 80 850 500]);
link1 = line([0 PosJoit(1,1)],[0 PosJoit(2,1)],...
             'Color','k','Marker','.','MarkerSize',10,'LineWidth',1.5);
link2 = line([PosJoit(1,1) PosJoit(3,1)],[PosJoit(2,1) PosJoit(4,1)],...
             'Color','k','Marker','.','MarkerSize',10,'LineWidth',1.5);
link3 = line([PosJoit(3,1) xac(1,1)],[PosJoit(4,1) xac(2,1)],...
             'Color','k','Marker','.','MarkerSize',10,'LineWidth',1.5);
%axis([-1 2 -0.5 1.5])
axis([-1 2 -0.5 1.5])
axis equal
xlabel('X (m)','fontname','Times');
ylabel('Y (m)','fontname','Times');
title('Trajectory with Max Manipulability','Fontname','Times');
hold on

for i=1:5:steps         % The points are so dense, so pick a point every 5 data.
    plot(xac(1,i),xac(2,i),'Marker','.','Color','r','MarkerSize',5)
    set(link1,'xdata',[0 PosJoit(1,i)],'ydata',[0 PosJoit(2,i)]);
    set(link2,'xdata',[PosJoit(1,i) PosJoit(3,i)],'ydata',[PosJoit(2,i) PosJoit(4,i)]);
    set(link3,'xdata',[PosJoit(3,i) xac(1,i)],'ydata',[PosJoit(4,i) xac(2,i)]);
    hold on
    
     if  i>1 && i < 2200 && mod(i,250)==1
         J = R3Jac(q(:,i));
         [U, S, V] = svd(J);              % Generate the ellipsoid coordinate --> ellipse in 2D 
         xdot(1,:) = S(1,1) * cos(pra);
         xdot(2,:) = S(2,2) * sin(pra);
         xdot = U*xdot;
        
         plot(0.08*xdot(1,:)+xac(1,i), 0.08*xdot(2,:)+xac(2,i),'Color','b','Linewidth',1.5);
         hold on
     end 
    pause(2*dt);
end

qstd = zeros(3, steps);
qstd(:,1) = [3*pi/4, -pi/2, pi/2]';
xacstd = zeros(2, steps);
PosJoitstd = zeros(4, steps);

for i=1:steps
    xacstd(:,i) = R3fk(qstd(:,i));
    PosJoitstd(:,i) = R3posjoit(qstd(:,i));
    xer = xd(:,i) - xacstd(:,i);
    J = R3Jac(qstd(:,i));
    Jri = Wpinv(J,I);
%   qdntemp = R3MaxManipty(1,q(:,i));
    qdot = Jri*(xd_d(:,i) + K*xer);  % + (I-Jri*J)*qdntemp;
    qstd(:,i+1) = qstd(:,i) + qdot*dt;
end

fig2 = figure('position',[50 80 850 500]);
link1 = line([0 PosJoitstd(1,1)],[0 PosJoitstd(2,1)],...
             'Color','k','Marker','.','MarkerSize',10,'LineWidth',1.5);
link2 = line([PosJoitstd(1,1) PosJoitstd(3,1)],[PosJoitstd(2,1) PosJoitstd(4,1)],...
             'Color','k','Marker','.','MarkerSize',10,'LineWidth',1.5);
link3 = line([PosJoitstd(3,1) xacstd(1,1)],[PosJoitstd(4,1) xacstd(2,1)],...
             'Color','k','Marker','.','MarkerSize',10,'LineWidth',1.5);
axis([-1 2 -0.5 1.5])
axis equal
xlabel('X (m)','fontname','Times');
ylabel('Y (m)','fontname','Times');
title('Standard Trajectory','Fontname','Times');
hold on

for i=1:5:steps         % The points are so dense, so pick a point every 5 data.
    plot(xacstd(1,i),xacstd(2,i),'Marker','.','Color','r','MarkerSize',5)
    set(link1,'xdata',[0 PosJoitstd(1,i)],'ydata',[0 PosJoitstd(2,i)]);
    set(link2,'xdata',[PosJoitstd(1,i) PosJoitstd(3,i)],'ydata',[PosJoitstd(2,i) PosJoitstd(4,i)]);
    set(link3,'xdata',[PosJoitstd(3,i) xacstd(1,i)],'ydata',[PosJoitstd(4,i) xacstd(2,i)]);
    hold on
    
     if  i>1 && i < 2200 && mod(i,250)==1 
         J = R3Jac(qstd(:,i));
         [U, S, V] = svd(J);
         xdot(1,:) = S(1,1) * cos(pra);
         xdot(2,:) = S(2,2) * sin(pra);
         xdot = U*xdot;
         %xdot = J*qunit;
         plot(0.08*xdot(1,:)+xacstd(1,i), 0.08*xdot(2,:)+xacstd(2,i),'Color','b','Linewidth',1.5);
         hold on
     end 
    pause(2*dt);
end
