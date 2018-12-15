%%% ENGG5402 Advanced Robotics HW5_3e&3f
close all
clear
clc

q0 = [0 pi 0.0001]';
dt = 0.001;
steps = 1/dt;
q = zeros(3, steps);
s = zeros(2, steps);
mpy = zeros(1, steps);
dyics_mpy = zeros(1, steps);
Y = zeros(1, steps);
q(:, 1) = q0;
K = diag([20 20 20]);
x_dot = [0 ; 2; 0];

for i = 1 : steps - 1
    q1 = q(1, i);
    q2 = q(2, i);
    q3 = q(3, i);
    q_1 = q1;
    q_12 = q1 + q2;
    q_123 = q_12 + q3;
    J = [-sin(q_1)-sin(q_12)-sin(q_123)     -sin(q_12)-sin(q_123)    -sin(q_123);
         cos(q_1)+cos(q_12)+cos(q_123)      cos(q_12)+cos(q_123)      cos(q_123);
         1                                     1                        1];
    x_act = [ cos(q_1)+cos(q_12)+cos(q_123);
              sin(q_1)+sin(q_12)+sin(q_123);
              q_123];
    x_dsr = [ -1;
              i*2/1000;
              pi];
    x_err = x_dsr - x_act;
    q_dot = pinv(J) * (x_dot + K * x_err);
    q(:, i+1) = q(:, i) + q_dot * dt;
    
    J_red = J(1:2, :);
    Jv = J_red * J_red';
    Y(1, i) = x_act(2, 1);
    mpy(1, i) = sqrt(det(Jv));
    k = inv(Jv);
    
    J_red1 = J(1:2, 1:2);
    J_P = pinv(J_red1);
    M = [3 + 2*cos(q2), 1 + cos(q2);
         1 + cos(q2), 1];
    KD = J_P' * (M') * M * J_P;
    dyics_mpy(1, i) = sqrt(det(inv(KD)));
    % s(:, i) = svd(inv(Jv));
end

figure('Position',[100, 100, 800, 500])
plot(Y(1:steps-1), mpy(1:steps-1), 'LineWidth', 1.5)
xlabel('y (m)', 'Fontname', 'Times', 'Fontsize', 12);
ylabel('$\bf{\sqrt{JJ^T}}$', 'Interpreter', 'LaTex', 'Fontsize', 10);
title('Velocity Manipulability', 'Fontname', 'Times', 'Fontsize', 12)

figure('Position',[100, 100, 800, 500])
plot(Y(1:steps-1), dyics_mpy(1:steps-1), 'LineWidth', 1.5)
xlabel('y (m)', 'Fontname', 'Times', 'Fontsize', 12);
ylabel('$\bf{\sqrt{{(J^\#M^TMJ^\#)}^{-1}}}$', 'Interpreter', 'LaTex', 'Fontsize', 10);
title('Dynamic Manipulability', 'Fontname', 'Times', 'Fontsize', 12)
