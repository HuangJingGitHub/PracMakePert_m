%%% MAEG5090 Topics in Robotics Assignment1: 2b %%%
clear
close all
clc

load('z_ref');
load('z_rot');
load('timeLen');
dT    = 0.005;
timeSeries  = 0:dT:timeLen;
timeStepNum = length(timeSeries);
footprint_x = 0.15;
footprint_y = 0.2;

robotMass  = 80;
gacl       = 9.8;
CoMHeight  = 1.2;
maxStride  = 0.8;
RQ_ratio = 1e-12;
A = [1  dT  dT^2/2  0 0 0;
     0  1   dT      0 0 0;
     0  0   1       0 0 0;
     0  0   0  1  dT  dT^2/2;
     0  0   0  0  1  dT;
     0  0   0  0  0  1];
B = [dT^3/6 0; dT^2/2 0; dT 0; 0 dT^3/6; 0 dT^2/2; 0 dT];
C = [1 0 CoMHeight / gacl 0 0 0;
     0 0 0 1 0 CoMHeight / gacl];

N = 10;
T = cell(N, 1);
S = cell(N, N);
C_N = cell(N, N);
for i = 1:N
    T{i, 1} = A^i;
    for j = 1:N
        C_N{i, j} = zeros(2, 6);
        if j <= i
            S{i, j} = A^(i-j) * B;
        else
            S{i, j} = zeros(6, 2);
        end
    end
    C_N{i, i} = C;
end
T = cell2mat(T);
S = cell2mat(S);
C_N = cell2mat(C_N);

Px = C_N * T;
Pu = C_N * S;
e = zeros(2, 2*N);
e(1, 1) = 1;
e(2, 2) = 1;
x_0 = [z_ref(1,1) 0 0 z_ref(2,1) 0 0]';
boundary = [footprint_x footprint_x footprint_y footprint_y]'*5;  % Set polygon boundary condition here
pxy = zeros(2, timeStepNum - N);
for i = 1:timeStepNum - N
    Z_REF = z_ref(:, i:i+N-1);
    Z_REF = Z_REF(:);
    rotAng = z_rot(i);
    R = [cos(rotAng) sin(rotAng); 
         -sin(rotAng) cos(rotAng)];
%    jerk = e * (-(Pu'*Pu + RQ_ratio * eye(2*N,2*N)) \ Pu' * (Px * x_0 - Z_REF));   %%% Analytical solution for 2a
    [A_ieqN, b_ieqN] = Ab_ieqN(A, B, C, R, x_0, Z_REF, boundary, N);
    H = Pu'*Pu + (1e-12)*eye(2*N,2*N);
    jerk = quadprog(H, Pu'*(Px * x_0 - Z_REF), A_ieqN, b_ieqN);
    jerk = e * jerk;
    x_0 = A * x_0 + B * jerk;
    pxy(:,i) = C * x_0;
end
figure('position',[120 95 1000 500])
plot(pxy(1,:))
hold on
plot(pxy(2,:))
% figure('position',[120 95 1000 500])
% x_ref = 3*sin(pi/40*timeSeries);
% y_ref = timeSeries/2;
% plot(timeSeries(N+1:end), pxy(2,:), 'LineWidth', 1.2)
% hold on
% plot(timeSeries, y_ref, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.2)
% xlabel('time (s)','Fontname','Times', 'FontSize', 12);
% ylabel('y','Fontname','Times', 'FontSize', 12);
% leng1 = legend('ZMP', 'CoM');
% set(leng1, 'Location', 'northeast', 'Orientation', 'horizontal');
