%%% Given a constant velocity of the end effector of a 2R plannar robot,
%%% calculate the command joint torques in a time interval. Combination
%%% of inverse kinematics and inverse dynamics (iterative Newton-Euler
%%% dynamics algorithm)
%%% Describtions in John Craig's book chapter6, page 211-212 (4th edition).
%%% HUANG Jing @CUHK MAE, August 2018

clear
close all
clc

%%% Robot configurations
DR     = pi/180;
g      = 9.81;
L1     = 1;
L2     = 0.5;
den    = 7806;               % Material density
widh   = 5e-2;
high   = 5e-2;
m      = [den*L1*widh*high den*L2*widh*high]';     % Mass of the two links

%%% Initial tensor of link in the frame with mass center as the origin
Intor1 = [m(1)*(widh^2 + high^2)/3 m(1)*(L1^2 + high^2)/3 m(1)*(L1^2 + widh^2)/3];
Intor2 = [m(2)*(widh^2 + high^2)/3 m(2)*(L2^2 + high^2)/3 m(2)*(L2^2 + widh^2)/3];
CIZ    = cell(2);
CIZ{1} = diag(Intor1);
CIZ{2} = diag(Intor2);

P(:,1) = [L1 0 0]';           % Coordinates of origins
P(:,2) = [L2 0 0]';
Pc(:,1) = [L1/2 0 high/2]';   % Coordinates of mass centers/GCs
Pc(:,2) = [L2/2 0 high/2]';
Z = [0 0 1]';

%%% Simulation configurations
Time = 1;
dt = 0.01;
t = 0:dt:Time;
Len = length(t);
x_d = [0 0.5]';
J = ones(3,2);    % Jacobian matrix in base frame is:
                  % J = [ -L1s1-L2s12 -L2s12;
                  %       L1c1+L2c12  L2c12;
                  %       1           1     ].
q0 = DR*[10 90]';
% Get the joint angle, angular velocity, angular acceleration at each time
% step using function AngleCal().
[q, qd, qdd, X] = AngleCal(2, L1, L2, q0, dt, Len, x_d);

w   = zeros(3,3);       % omega, column vector
wd  = zeros(3,3);       % time derivative of omega
vd  = zeros(3,3);       % velocity derivative/acceleration of origins
vcd = zeros(3,3);       % accelerations of mass centers
F   = zeros(3,3);       % net force
N   = zeros(3,3);       % net torque
f   = zeros(3,3);       % required commanded force
n   = zeros(3,3);       % required commanded torque
tau = zeros(2,1);       % Z compoent of n
tau1= zeros(2,Len); 

%%% Initialize configurations of the first link, i.e. base.
w(:,1)  = [0 0 0]';   
wd(:,1) = [0 0 0]';
vd(:,1) = [0 0 0]';
vcd(:,1)= [0 g 0]';

for loop = 2:Len
    %%% Outward iterations to compute velocities, accelerations, the net forces
    for olink = 1:2
        w(:,olink+1)   = rotz(-q(olink,loop))*w(:,olink) + qd(olink,loop)*Z;
        wd(:,olink+1)  = rotz(-q(olink,loop))*wd(:,olink)...
                         + cross(rotz(-q(olink,loop))*w(:,olink), qd(olink,loop)*Z)+ qdd(olink,loop)*Z;
        vd(:,olink+1)  = rotz(-q(olink,loop))*(cross(wd(:,olink),P(:,olink))...
                         + cross(w(:,olink),cross(w(:,olink),P(:,olink))) + vd(:,olink));
        vcd(:,olink+1) = cross(wd(:,olink+1), Pc(:,olink))...
                         + cross(w(:,olink+1),cross(w(:,olink+1),Pc(:,olink))) + vd(:,olink+1);
        F(:,olink+1)   = m(olink) * vcd(:,olink+1);
        N(:,olink+1)   = CIZ{olink}*wd(:,olink+1) + cross(w(:,olink+1),(CIZ{olink}*w(:,olink+1)));
    end
    
    %%% Inward interations to compute actuating forces and torques.
    for ilink = 3:-1:2
            f(:,ilink-1) = rotz(q(ilink-1,loop))*f(:,ilink) + F(:,ilink);
            n(:,ilink-1) = N(:,ilink) + rotz(q(ilink-1,loop))*n(:,ilink)...
                           + cross(Pc(:,ilink-1),F(:,ilink))...
                           + cross(P(:,ilink-1),rotz(q(ilink-1,loop))*f(:,ilink));
        tau(ilink-1) = n(:,ilink-1)'*Z;
    end
    tau1(:,loop) = tau;
end

%%% Visualize results
%%% (1) joint angle vs time
figure('position',[0 -250 1450 850]);
plot(t(2:Len), q(1,2:Len),'color','b','linewidth',2);
hold on
plot(t(2:Len), q(2,2:Len),'color','k','linewidth',2);
xlabel('$Time$ $(s)$','interpreter','latex','fontsize',15);
ylabel('$Joint$ $Angules$ $(rad)$','interpreter','latex','fontsize',15);
h1=legend('${\theta_1}$','${\theta_2}$',0);
set(h1,'interpreter','latex','Box','off','fontsize',18);

%%% (2) joint velocities vs time
figure('position',[0 -250 1450 850]);
plot(t(2:Len), qd(1,2:Len),'color','b','linewidth',2);
hold on
plot(t(2:Len), qd(2,2:Len),'color','k','linewidth',2);
xlabel('$Time$ $(s)$','interpreter','latex','fontsize',15);
ylabel('$Joint$ $Velocity$ $(rad/s)$','interpreter','latex','fontsize',15);
h1=legend('$\dot{\theta_1}$','$\dot{\theta_2}$',0);
set(h1,'interpreter','latex','Box','off','fontsize',18);

%%% (3) joint accelerations vs time
%%% As can be seen, the initial acceleration from static state to 
%%% motion could be quite large.
figure('position',[0 -250 1450 850]);
plot(t(2:Len), qdd(1,2:Len),'color','b','linewidth',2);
hold on
plot(t(2:Len), qdd(2,2:Len),'color','k','linewidth',2);
xlabel('$Time$ $(s)$','interpreter','latex','fontsize',15);
ylabel('$Joint$ $Accelerations$ $(rad/s^2)$','interpreter','latex','fontsize',15);
h1=legend('$\ddot{\theta_1}$','$\ddot{\theta_2}$',0);
set(h1,'interpreter','latex','Box','off','fontsize',18);

%%% (4) cartesian components vs time
figure('position',[0 -250 1450 850]);
plot(t(2:Len), X(1,2:Len),'color','b','linewidth',2);
hold on
plot(t(2:Len), X(2,2:Len),'color','k','linewidth',2);
hold on
plot(t(2:Len), X(3,2:Len),'color','g','linewidth',2);
xlabel('$Time$ $(s)$','interpreter','latex','fontsize',15);
ylabel('$Cartesian$ $Components$','interpreter','latex','fontsize',15);
h1=legend('$x$','$y$','$\phi$',0);
set(h1,'interpreter','latex','Box','off','fontsize',18);

%%% (5) joint torques vs time
figure('position',[0 -250 1450 850]);
plot(t(3:Len), tau1(1,3:Len),'color','b','linewidth',2);
hold on
plot(t(3:Len), tau1(2,3:Len),'color','k','linewidth',2);
xlabel('$Time$ $(s)$','interpreter','latex','fontsize',15);
ylabel('$Joint$ $Torques$ $N{\cdot}m$','interpreter','latex','fontsize',15);
h1=legend('${\tau_1}$','${\tau_2}$',0);
set(h1,'interpreter','latex','Box','off','fontsize',18);
        
