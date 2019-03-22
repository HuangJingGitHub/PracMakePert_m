%%% ENGG5090 Topics in Roboatics Dynamics Assignment1 1a %%%
clear
close all
clc

linkNum  = 7;
linkMass = 0.2;
Ic = diag([0.0005 0 0.0005]);
gravity = -9.8;

timeLen     = 6;
timeStep    = 0.001;
timeSeries  = 0:timeStep:timeLen;
stepNum     = length(timeSeries);

q_t   = zeros(linkNum, stepNum);
q_dt  = zeros(linkNum, stepNum);
q_ddt = zeros(linkNum, stepNum);

for i = 1:linkNum
    q_t(i,:)    = 0.6 * sin(pi*timeSeries + (i-1)*pi/3);
    q_dt(i,:)   = 0.6 * pi * cos(pi*timeSeries + (i-1)*pi/3);
    q_ddt(i,:)  = -0.6 * pi * pi * sin(pi*timeSeries + (i-1)*pi/3);
end

r = [0 0 -0.025; 0 0 0; 0.025 0 0];
It = zeros(6, 6);
It(1:3, 1:3) = Ic - linkMass * r * r;
It(1:3, 4:6) = linkMass * r;
It(4:6, 1:3) = linkMass * r';
It(4:6, 4:6) = linkMass * eye(3,3); 

v = zeros(6, linkNum+1);
a = zeros(6, linkNum+1);
a(6, 1) = -gravity;
f = zeros(6, linkNum+1);
tau = zeros(7, stepNum);
h = [1 0 0 0 0 0.1]';

for i = 1:stepNum
    for n = 1:linkNum
        h           = [1 0 0 0 -0.1*sin(q_t(n, i)) 0.1*cos(q_t(n, i))]';
        [X_M, X_F]  = spatialTrans(q_t(n, i), [0, 0.1, 0]');
        v(:, n+1)   = X_M * v(:, n) + h * q_dt(n, i);
        a(:, n+1)   = X_M * a(:, n) + spatialCrossM(v(:, n+1), h) * q_dt(n, i)+ h * q_ddt(n, i);
        f(:, n+1)   = It * a(:, n+1) + spatialCrossF(v(:, n+1), It*v(:, n+1));
    end
    for n = linkNum+1:-1:2
        h           = [1 0 0 0 -0.1*sin(q_t(n-1, i)) 0.1*cos(q_t(n-1, i))]';
        [~, X_F]    = spatialTrans(q_t(n-1, i), [0, 0.1, 0]');
        tau(n-1, i) = h'*f(:, n);
        f(:, n-1)   = f(:, n-1) + X_F \ f(:, n);
    end 
%     v = zeros(6, linkNum+1);
%     a = zeros(6, linkNum+1);
%     a(6, 1) = -gravity;
%     f = zeros(6, linkNum+1);
end

figure
plot(timeSeries, tau(1,:));
hold on
plot(timeSeries, tau(2,:));
hold on
plot(timeSeries, tau(3,:));
hold on
plot(timeSeries, tau(4,:));
hold on
plot(timeSeries, tau(5,:));
hold on
plot(timeSeries, tau(6,:));
hold on
plot(timeSeries, tau(7,:));