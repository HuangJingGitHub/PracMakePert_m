%%% Trajectory generation. Introduction to Robotics John J. Craig. Chapter
%%% 7. Problem descriptions are on page244, 4th edition. Angles in unit deg.
%%% HUANG Jing @CUHK MAE, August 2018.

%%% a) Third-order polynomial:
clear
close all;

the    = [120 60];
the_d  = [0 0];
the_dd = [0 0];
tf     = 1;
dt     = 1/50;
t      = 0:dt:tf;

% coefficients matrix for angle. Refer to chapter 7 of Craig's book.
a1(1) = the(1);
a1(2) = 0; 
a1(3) = 3*(the(2) - the(1)) / (tf^2);
a1(4) = -2*(the(2) - the(1)) / (tf^3);
% coefficients matrix for angular velocity and acceleration
av1 = [ a1(2) 2*a1(3) 3*a1(4) ];
aa1 = [ 2*a1(3) 6*a1(4)];

%%% Vectorization programming is much preferred than loop based on single 
%%% variable for faster speed and compacter structure. The powers of time are
%%% stored in T.
T(1,:) = ones(1, size(t,2));
T(2,:) = t;
T(3,:) = t.^2;
T(4,:) = t.^3;
T(5,:) = t.^4;
T(6,:) = t.^5;

q1    = a1*T(1:4,:);     % angle
q_d1  = av1*T(1:3,:);    % angular velocity    
q_dd1 = aa1*T(1:2,:);    % angular acceleration


%%% b) Fifth-order/quintic polynomial:
%%% similar procedure
a2(1) = the(1);
a2(2) = the_d(1); 
a2(3) = the_dd(1)/2;
a2(4) = (20*the(2)-20*the(1)-(8*the_d(2)+12*the_d(1))*tf-(3*the_dd(1)-the_dd(2))*tf)...
        /(2*tf^3);
a2(5) = (30*the(1)-30*the(2)+(14*the_d(2)+16*the_d(1))*tf+(3*the_dd(1)-2*the_dd(2))*tf)...
        /(2*tf^4);
a2(6) = (12*the(2)-12*the(1)+(6*the_d(2)+6*the_d(1))*tf-(the_dd(1)-the_dd(2))*tf^2)...
        /(2*tf^5);

av2 = [ a2(2) 2*a2(3) 3*a2(4) 4*a2(5) 5*a2(6)];
aa2 = [ 2*a2(3) 6*a2(4) 12*a2(5) 20*a2(6) ];

q2 = a2*T;
q_d2 = av2*T(1:5,:);
q_dd2 =aa2*T(1:4,:);

%%% c)Two cubic polynomials
a31(1) = 60;          % The coefficient matrix of the first interval is labeled as a31.
a31(2) = 0;           % The coefficient matrix of the second interval is labeled as a32.
a32(1) = 120;

A = [ 1 1 0 0 0;      % 3 of these coefficients are easy to get. The left 5 are solved 
      0 0 1 1 1;      % through linear equations AX=b. The derevation of the equations 
      0 0 1 2 3;      % is easy using the constraints.
      2 3 -1 0 0;     % X = [a31(3) a31(4) a32(2) a32(3) a32(4)]'.
      2 6 0 -2 0];
b = [60 -90 0 0 0]';
X = A\b;
X = X';
a31 = [a31 X(1:2)];
a32 = [a32 X(3:5)];

av31 = [a31(2) 2*a31(3) 3*a31(4)];
aa31 = [2*a31(3) 6*a31(4)];

av32 = [a32(2) 2*a32(3) 3*a32(4)];
aa32 = [2*a32(3) 6*a32(4)];

q31    = a31*T(1:4,:);
q_d31  = av31*T(1:3,:);
q_dd31 = aa31*T(1:2,:);

q32    = a32*T(1:4,:);
q_d32  = av32*T(1:3,:);
q_dd32 = aa32*T(1:2,:);

q3    = [q31 q32(2:end)];         % Concatenate the data of the 1st and 2nd interval.
q_d3  = [q_d31 q_d32(2:end)];     % Details: (1)The end entry of q31 and the first entry of q32
q_dd3 = [q_dd31 q_dd32(2:end)];   % are the same. Keep one in the result to avoid overlap.
t3 = 0:dt:dt*(size(q3,2)-1);      % (2) Pay attention to the entry number of t3 created.
                                  % t3 = 0:dt:dt*(size(q3,2)-1) not t3 =
                                  % 0:dt:dt*size(q3,2)
%%% Plot part. figure1
figure('position',[50 -200 1200 800]);
H11 = subplot(3,1,1);
plot(t,q1,'linewidth',2)
ylabel('$\theta$','interpreter','latex','fontsize',18);
set(gca,'XTick',[]); 
title('\it{Cubic Polynomial Trajectory}','fontname','Times','fontsize',16);
H12 = subplot(3,1,2);
plot(t,q_d1,'linewidth',2)
ylabel('$\dot{\theta}$','interpreter','latex','fontsize',18);
set(gca,'XTick',[]); 
H13 = subplot(3,1,3)
plot(t,q_dd1,'linewidth',2);
ylabel('$\ddot{\theta}$','interpreter','latex','fontsize',18);
xlabel('\it{Time (s)}','fontname','Times','fontsize',16);

%%% figure2
figure('position',[50 -200 1200 800]);
H21 = subplot(3,1,1);
plot(t,q2,'linewidth',2)
ylabel('$\theta$','interpreter','latex','fontsize',18);
set(gca,'XTick',[]);
title('\it{Quintic Polynomial Trajectory}','fontname','Times','fontsize',16);
H22 = subplot(3,1,2);
plot(t,q_d2,'linewidth',2)
ylabel('$\dot{\theta}$','interpreter','latex','fontsize',18);
set(gca,'XTick',[]); 
H23 = subplot(3,1,3);
plot(t,q_dd2,'linewidth',2);
ylabel('$\ddot{\theta}$','interpreter','latex','fontsize',18);
xlabel('\it{Time (s)}','fontname','Times','fontsize',16);

%%% figure3
figure('position',[50 -200 1200 800]);
H31 = subplot(3,1,1);
plot(t3,q3,'linewidth',2)
ylabel('$\theta$','interpreter','latex','fontsize',18);
set(gca,'XTick',[]); 
title('\it{Cubic Polynomial Trajectory with Via Point}','fontname','Times','fontsize',16);
H32 = subplot(3,1,2);
plot(t3,q_d3,'linewidth',2)
ylabel('$\dot{\theta}$','interpreter','latex','fontsize',18);
set(gca,'XTick',[]); 
H33 = subplot(3,1,3);
plot(t3,q_dd3,'linewidth',2);
ylabel('$\ddot{\theta}$','interpreter','latex','fontsize',18);
xlabel('\it{Time (s)}','fontname','Times','fontsize',16);
