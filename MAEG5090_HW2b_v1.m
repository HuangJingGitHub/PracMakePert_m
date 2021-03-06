%%% MAEG5090 Topics in Robotics Assignment1: 2b %%%
clear
close all
clc

load('z_ref');
load('timeLen');
dT    = 0.005;
timeSeries  = 0:dT:timeLen;
timeStepNum     = length(timeSeries);

robotMass  = 80;
gacl       = 9.8;
CoMHeight  = 1.2;
maxStride  = 0.8;
RQ_ratio = 1e-12;
A = [1 dT dT^2/2; 0 1 dT; 0 0 1];
B = [dT^3/6; dT^2/2; dT];
C = [1 0 CoMHeight / gacl];

N = 10;
T = cell(N, 1);
S = cell(N, N);
C_N = cell(N, N);
for i = 1:N
    T{i, 1} = A^i;
    for j = 1:N
        C_N{i, j} = zeros(1, 3);
        if j <= i
            S{i, j} = A^(i-j) * B;
        else
            S{i, j} = zeros(3, 1);
        end
    end
    C_N{i, i} = C;
end
T = cell2mat(T);
S = cell2mat(S);
C_N = cell2mat(C_N);

Px = C_N * T;
Pu = C_N * S;
e = zeros(1, N)';
e(1) = 1;
x_0 = [z_ref(1,1) 0 0]';
px = zeros(1, timeStepNum);
px(1:N) = 0;
for i = 1:timeStepNum - N
    Z_REF = z_ref(1, i:i+N-1)';  % x direction
%     Z_REF = z_ref(2, i:i+N-1)'; % y direction
    jerk = e' * (-(Pu'*Pu + RQ_ratio * eye(N,N)) \ Pu' * (Px * x_0 - Z_REF));
    x_0 = A * x_0 + B * jerk;
    px(1,i+N) = C * x_0;
end
figure('position',[120 95 1000 500])
x_ref = 3*sin(pi/40*timeSeries);
y_ref = timeSeries/2;
plot(timeSeries, px, 'LineWidth', 1.2)
hold on
plot(timeSeries, x_ref, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.2)
xlabel('time (s)','Fontname','Times', 'FontSize', 12);
ylabel('y','Fontname','Times', 'FontSize', 12);
leng1 = legend('ZMP', 'CoM');
set(leng1, 'Location', 'northeast', 'Orientation', 'horizontal');