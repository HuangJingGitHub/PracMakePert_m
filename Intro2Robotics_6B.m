%%% Inverse-dynamics solution for the plannar 3-DOF, 3R robot for a motion snapshot
%%% in time only. The iterative Newton-Euler dynamics algorithm.
%%% Describtions on John Craig's book chapter6, page 212 (4th edition).
clear
close all
clc

%%% Robot configurations
DR = pi/180;
g = 9.81;
link = 3;
L1 = 4;
L2 = 3;
L3 = 2;
m  = [20 15 10]';
CIZ = cell(3);           % The use of cell array is not favored in large-sacle computation.
CIZ{1} = zeros(3,3);     % And use of matrix is also feasible with other index way.
CIZ{2} = zeros(3,3);     % Inertia tensor
CIZ{3} = zeros(3,3);
CIZ{1}(3,3) = 0.5;
CIZ{2}(3,3) = 0.2;
CIZ{3}(3,3) = 0.1;
P(:,1) = [L1 0 0]';      % Coordinates of origins
P(:,2) = [L2 0 0]';
P(:,3) = [L3 0 0]';
Pc(:,1) = [L1/2 0 0]';   % Coordinates of mass centers/GCs
Pc(:,2) = [L2/2 0 0]';
Pc(:,3) = [L3/2 0 0]';
Z = [0 0 1]';

%%% Angles, angular velocities, accelerations
q   = DR * [10 20 30]';
qd  = [1 2 3]';
qdd = [0.5 1 1.5]';

w   = zeros(3,4);       % omega, column vector
wd  = zeros(3,4);       % time derivative of omega
vd  = zeros(3,4);       % velocity derivative/acceleration of origins
vcd = zeros(3,4);       % accelerations of mass centers
F   = zeros(3,4);       % net force
N   = zeros(3,4);       % net torque
f   = zeros(3,4);       % required commanded force
n   = zeros(3,4);       % required commanded torque
tau = zeros(3,1);       % Z compoent of n
%%% Initialize configurations of the first link, i.e. base
w(:,1)  = [0 0 0]';   
wd(:,1) = [0 0 0]';
vd(:,1) = [0 0 0]';
vcd(:,1)= [0 g 0]';

%%% Outward iterations to compute velocities, accelerations, the net forces
%%% and torques acting on the links. Refer to John Criig's book p187-188.
for loop = 1:1:link
    w(:,loop+1)  = rotz(-q(loop))*w(:,loop) + qd(loop)*Z;
    wd(:,loop+1) = rotz(-q(loop))*wd(:,loop) + cross(rotz(-q(loop))*w(:,loop), qd(loop)*Z)...
                   + qdd(loop)*Z;
    vd(:,loop+1) = rotz(-q(loop))*(cross(wd(:,loop),P(:,loop)) + cross(w(:,loop),cross(w(:,loop),P(:,loop)))...
                   + vd(:,loop));
    vcd(:,loop+1) = cross(wd(:,loop+1), Pc(:,loop)) + cross(w(:,loop+1),cross(w(:,loop+1),Pc(:,loop)))...
                   + vd(:,loop+1);
    F(:,loop+1) = m(loop) * vcd(:,loop+1);
    N(:,loop+1) = CIZ{loop}*wd(:,loop+1) + cross(w(:,loop+1),(CIZ{loop}*w(:,loop+1)));
end

%%% Inward interations to compute actuating forces and torques.
%%% Pay attention to the indices. The end-effector is free.
for loop = 4:-1:2
    f(:,loop-1) = rotz(q(loop-1))*f(:,loop) + F(:,loop);
    n(:,loop-1) = N(:,loop) + rotz(q(loop-1))*n(:,loop) +cross(Pc(:,loop-1),F(:,loop))...
                  + cross(P(:,loop-1),rotz(q(loop-1))*f(:,loop));
    tau(loop-1) = n(:,loop-1)'*Z;
end
    
