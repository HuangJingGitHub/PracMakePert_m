%%% ENGG5402 HW2_1
clc
clear
close all

syms q1 q2 q3
D2R = pi/180;
DH = [
  % type(i)  alpha(i-1)  a(i-1)  d(i)  theta(i)  type == 1, revolute joint
  % ======   =========   =====   ====  ========  type == 0, primastic joint
       1          0        0       1      q1;
       1          -pi/2    0       0      q2;
       1          0        1       0      q3;
       1          pi/2     1       0      0 ];

n = size(DH,1);
T = eye(4);
Z = sym(zeros(3,n));
P = sym(zeros(3,n));
J = sym(zeros(6,n-1));
for i=1:n
    alp = DH(i,2);
    a   = DH(i,3);
    d   = DH(i,4);
    the = DH(i,5);
    Ti = [cos(the)          -sin(the)         0         a;
          sin(the)*cos(alp) cos(the)*cos(alp) -sin(alp) -sin(alp)*d;
          sin(the)*sin(alp) cos(the)*sin(alp) cos(alp)  cos(alp)*d;
          0                 0                 0         1];
    T = T*Ti;
    Z(:,i) = T(1:3,3);
    P(:,i) = T(1:3,4);
    if i == 1
        T0_1 = T;
    elseif i == 2
        T0_2 = T;
    elseif i == 3
        T0_3 = T;
    end 
end

for i = 1:n-1
    type = DH(i,1);
    if type
        J(1:3,i) = cross(Z(:,i), P(:,end)-P(:,i));
        J(4:6,i) = Z(:,i);
    else
        J(1:3,i) = Z(:,i);
        J(4:6,i) = [0 0 0]';
    end
end

q2_cig = -85:20:-5;                   % Set 5 representive postures.      
cig = D2R * [-45 -45 -45 -45 -45;
             q2_cig;                  % Pay attention to the sign of q2 and q3.
             -q2_cig*2];
pra = linspace(0, 2*pi, 100);
qdot = zeros(3,100);                  % Only joint2 and 3 can move.
qdot(2,:) = cos(pra);                 % qdot'*qdot = 1
qdot(3,:) = sin(pra);
tau = qdot;                           % Only consider force in X-Z plane, tau1 = 0.

fig1 = figure('Position',[50 80 850 500]);
fig2 = figure('Position',[50 80 850 500]);

for i=1:5
    Jq = double(subs(J,[q1 q2 q3],cig(:,i)'));     % Jq is the Jacobian at the specific posture.
    Pq = double(subs(P,[q1 q2 q3],cig(:,i)'));     % Pq stores the position of frames' origin to draw linkages.
    Pq(2,:) = sqrt(Pq(1,:).^2 + Pq(2,:).^2);       % The plot plane is X-Z plane, transform the X,Y coordinates.
    xdot = Jq*qdot;
    xdot = xdot(1:3,:);                            % Only the Cartesian velocities are considered. Only the first 3 rows of xdot are needed.
    xdot(2,:) = sqrt(2)*xdot(1,:);                 % Also velocity in X,Y axial direction need to be transformed to X-Z plane
                                                   % by geometric
                                                   % relationship.
    Jqt = Jq';                                     
    Jqt = Jqt(:,1:3);
    F = Jqt\tau;                                   % Since only force no torque in F components, J' must be reduced since inverse is used here.
    F(2,:) = sqrt(2)*F(1,:);                       % Coordinates transformation.
    
    figure(fig1)
    plot(0.3*xdot(2,:)+Pq(2,4),0.3*xdot(3,:)+Pq(3,4),'--k','Linewidth',1.5);
    l1 = line([0 Pq(2,2)],[0,Pq(3,2)],...
              'Color','k','Marker','.','MarkerSize',10, 'LineWidth',1.5);
    l2 = line([Pq(2,2) Pq(2,3)],[Pq(3,2) Pq(3,3)],...
              'Color','k','Marker','.','MarkerSize',10, 'LineWidth',1.5);
    l3 = line([Pq(2,3) Pq(2,4)],[Pq(3,3) Pq(3,4)],...
              'Color','k','Marker','.','MarkerSize',10, 'LineWidth',1.5);
    hold on
    if i==5
        xlabel('X','fontname','Times');
        ylabel('Z','fontname','Times');
        title('Velocity Manipulability Ellipses','fontname','Times');
    end
    
    figure(fig2)
    plot(0.1*F(2,:)+Pq(2,4),0.1*F(3,:)+Pq(3,4),'--k','linewidth',1.5);
    l4 = line([0 Pq(2,2)],[0,Pq(3,2)],...
              'Color','k','Marker','.','MarkerSize',10, 'LineWidth',1.5);
    l5 = line([Pq(2,2) Pq(2,3)],[Pq(3,2) Pq(3,3)],...
              'Color','k','Marker','.','MarkerSize',10, 'LineWidth',1.5);
    l6 = line([Pq(2,3) Pq(2,4)],[Pq(3,3) Pq(3,4)],...
              'Color','k','Marker','.','MarkerSize',10, 'LineWidth',1.5);
    hold on
    if i==5
        xlabel('X','fontname','Times');
        ylabel('Z','fontname','Times');
        title('Force Manipulability Ellipses','fontname','Times');
    end    
end
    