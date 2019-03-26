%%% ENGG5090 Topics in Roboatics Dynamics Assignment1 1b %%%
clear
close all
clc

linkNum  = 7;
links    = 1:7;
linkMass = 0.2;
Ic = diag([0.0005 0 0.0005]);
gravity = -9.8;

timeLen     = 6;
timeStep    = 0.001;
timeSeries  = 0:timeStep:timeLen;
stepNum     = length(timeSeries);

r = [0 0.075 0]';
r_mat = vector2Mat(r);
Ii = zeros(6, 6);
Ii(1:3, 1:3) = Ic - linkMass * r_mat * r_mat;
Ii(1:3, 4:6) = linkMass * r_mat;
Ii(4:6, 1:3) = linkMass * r_mat';
Ii(4:6, 4:6) = linkMass * eye(3,3);

v = zeros(6, linkNum+1);
a = zeros(6, linkNum+1);
a(6, 1) = -gravity;
f = zeros(6, linkNum+1);
%tau = zeros(linkNum, stepNum);
h = [1 0 0 0 0 0]';

load('test_torque.mat');

qt = zeros(linkNum, stepNum);
qt(:,1) = 0.6 * sin((links - 1) * pi/3 );
qdt = zeros(linkNum, stepNum);
qdt(:, 1) = 0.6 * pi * cos((links - 1) * pi/3 );
qddt = zeros(linkNum, stepNum);

for i = 1:stepNum
    q_t    = 0.6 * sin( pi * timeSeries(i) + (links - 1) * pi/3 );
    q_dt   = 0.6 * pi * cos(pi * timeSeries(i) + (links - 1) * pi/3 );
    M = CRBA(linkNum, Ii, h, q_t, 0.1);
    tau_noacl = Citem(q_t, q_dt, linkNum, h, Ii);    
     
%     M = CRBA(linkNum, Ii, h, qt(:, i), 0.1);
%     tau_noacl = Citem(qt(:, i), qdt(:, i), linkNum, h, Ii);
    q_acl = M \ (tau(:,i) - tau_noacl);
    qddt(:, i) = q_acl;
    qdt(:, i+1) = qdt(:, i) + q_acl * timeStep;
    qt(:, i+1) = qt(:, i) + qdt(:, i) * timeStep; % + 0.5 * q_acl * timeStep^2;
end

figure
plot(qt(2,:))
hold on
plot(qdt(2,:))
hold on
plot(qddt(2,:))


function M = CRBA(linkNum, Ii, hi, q_t, linkLength)   %%% Composite Rigid Body Algorithm
    fci = zeros(6, linkNum, linkNum);
    Ici = zeros(6, 6, linkNum);    
    M = zeros(linkNum, linkNum);

    for i = 1:linkNum
        Ici(:,:,i) = Ii;
    end
    
    for i = linkNum:-1:1
        fci(:, i, i) = Ici(:,:,i) * hi;
        for j = i:linkNum
            M(i, j) = hi' * fci(:,i, j);
            M(j, i) = M(i, j);
        end
        if i ~= 1
            [X_M, X_F] = spatialTrans(-q_t(i), [0 -linkLength*cos(q_t(i)) linkLength*sin(q_t(i))]');
            Ici(:,:,i-1) = Ici(:,:,i-1) + X_F * Ici(:,:,i) / X_M;
            for j = i:linkNum
                fci(:, i-1, j) = X_F * fci(:, i, j);
            end
        end
    end
end

function tau_noacl = Citem(q_t, q_dt, linkNum, h, Ii)
    tau_noacl = zeros(linkNum, 1);
    q_ddt  = zeros(linkNum, 1);
    
    v = zeros(6, linkNum+1);
    a = zeros(6, linkNum+1);
    a(6, 1) = 9.8;
    f = zeros(6, linkNum+1);

    [X_M, ~]  = spatialTrans(q_t(1), [0, 0, 0]');
    v(:, 2)   = X_M * v(:, 1) + h * q_dt(1);
    a(:, 2)   = X_M * a(:, 1) + spatialCrossM(v(:, 2), h) * q_dt(1)+ h * q_ddt(1);
    f(:, 2)   = Ii * a(:, 2) + spatialCrossF(v(:, 2), Ii*v(:, 2));    
    for n = 2:linkNum
        [X_M, ~]  = spatialTrans(q_t(n), [0, 0.1, 0]');
        v(:, n+1)   = X_M * v(:, n) + h * q_dt(n);
        a(:, n+1)   = X_M * a(:, n) + spatialCrossM(v(:, n+1), h) * q_dt(n)+ h * q_ddt(n);
        f(:, n+1)   = Ii * a(:, n+1) + spatialCrossF(v(:, n+1), Ii*v(:, n+1));
    end
    for n = linkNum+1:-1:2
        [~, X_F]    = spatialTrans(q_t(n-1), [0, 0.1, 0]');
        tau_noacl(n-1) = h'*f(:, n);
        f(:, n-1)   = f(:, n-1) + X_F \ f(:, n);
    end 
end