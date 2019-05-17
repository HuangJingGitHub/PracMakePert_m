%%% MAEG5090 Topics in Robotics HW2.a %%%
%%% huangjingonly@gmail.com %%%
clear 
close all
clc
% addpath('..')

linkNum  = 7;          
links    = 1:linkNum;
linkLength = 0.1 * ones(7, 1);
linkMass = 0.2;
Ic = diag([0.0005 0 0.0005]);
gravity = -9.8;

r = [0 0.075 0]';
r_mat = vector2Mat(r);
Ii = zeros(6, 6);
Ii(1:3, 1:3) = Ic - linkMass * r_mat * r_mat;
Ii(1:3, 4:6) = linkMass * r_mat;
Ii(4:6, 1:3) = linkMass * r_mat';
Ii(4:6, 4:6) = linkMass * eye(3,3);
h = [1 0 0 0 0 0]';

period = 2;
Time = 2;
dT = 0.001;
timeSeries = 0:dT:Time;
stepNum = length(timeSeries);

y_center = 0.5;
z_center = 0;
radius   = 0.2;
omega    = 2 * pi / period;
x_desire     = [ y_center + radius * cos(omega * timeSeries);
                 z_center + radius * sin(omega* timeSeries)];
xdot_desire  = [ -radius * omega * sin(omega * timeSeries);
                  radius * omega * cos(omega * timeSeries)];
xddot_desire = [ -radius * omega^2 * cos(omega * timeSeries);
                 -radius * omega^2 * sin(omega * timeSeries)];
x_errorNorm = zeros(stepNum, 1);            
q = zeros(linkNum, stepNum);
qdot = zeros(linkNum, stepNum);


%%% Control Mode %%%
% Mode 1: velocity-based control
% Mode 2: acceleration-based control
% Mode 3: force control
% Mode 4: hierarchy multi-task control 
% Mode 5: weighted multi-task control

Mode = 3;

%%% Velocity-based Control with Joint Velocity Integration %%%
if Mode == 1
Kp = 10 * eye(2);
alpha = 10;
Kqd = 1e-2 * eye(linkNum);
Kqp = 1e-2 * eye(linkNum);
q_ref_previous = q(:,1);
qdot_ref_previous = zeros(linkNum, 1);
x_actual = zeros(2, stepNum);
tau = zeros(linkNum, stepNum);
q(5, 1) = 0.1;
for i = 1:stepNum - 1
    M = CRBA(linkNum, Ii, h, q(:, i), linkLength);
    J = Jacobian_nR(linkNum, q(:, i), linkLength);    
    x_actual(:, i) = FK_nR(linkNum, q(:, i), linkLength); 
    x_errorNorm(i) = norm(x_actual(:, i) - x_desire(:,i));
    xdot_ref   = xdot_desire(:, i) + Kp * (x_desire(:,i) - x_actual(:, i));
    dg         = q(:, i);
    qdot_ref   = pinv(J) * xdot_ref - alpha * (eye(linkNum) - pinv(J) * J) * dg;
    qddot_ref  = (qdot_ref - qdot_ref_previous) / dT;
%     q_ref      = q_ref_previous + qdot_ref_previous * dT; 
%%%     Sensitive to q_ref.
    q_ref = q(:, i);  
    qdot_ref_previous = qdot_ref;
    q_ref_previous    = q_ref;
    
    tau(:, i) = ID(q_ref, qdot_ref, qddot_ref, linkNum, h, Ii) + Kqd*(qdot_ref - qdot(:, i)) + Kqp*(q_ref - q(:, i));    
    qddot = M \ (tau(:, i) - ID(q(:,i), qdot(:,i), zeros(linkNum, 1), linkNum, h, Ii));
    qdot(:, i+1) = qdot(:, i) + qddot * dT;
    q(:, i+1) = q(:, i) + qdot(:, i) * dT + qddot * dT^2 / 2;
end
end
% figure
% plot(x_actual(1,:), x_actual(2,:))

%%% Acceleration-based Control %%%
if Mode == 2
Kd = 500 * eye(2);
Kp = 500 * eye(2);
alpha = 10;
KN = eye(linkNum);

for i = 1:stepNum
    M = CRBA(linkNum, Ii, h, q(:, i), linkLength);
    J = Jacobian_nR(linkNum, q(:, i), linkLength); 
    J_dot = Jacobian_dot_nR(linkNum, q(:, i), qdot(:, i), linkLength);
    x_actual(:, i) = FK_nR(linkNum, q(:, i), linkLength);
    x_errorNorm(i) = norm(x_actual(:, i) - x_desire(:,i));    
    xdot_actual    = J * qdot(:, i);
    xddot_ref      = xddot_desire(:, i) + Kd * (xdot_desire(:, i) - xdot_actual) + Kp * (x_desire(:, i) - x_actual(:, i));
    
    dg = q(:, i);
    dg_dot = qdot(:, i);
    eN = (eye(linkNum) - pinv(J)*J) * (-alpha*dg - qdot(:,i));
    nullVec = (eye(linkNum) - pinv(J)*J) * (-alpha*dg_dot + KN*eN);
    tau(:, i) = M * (pinv(J) * (xddot_ref - J_dot * qdot(:, i)) + nullVec) + ID(q(:, i), qdot(:, i), zeros(linkNum, 1), linkNum, h, Ii);    
    qddot = M \ (tau(:, i) - ID(q(:,i), qdot(:,i), zeros(linkNum, 1), linkNum, h, Ii));
    qdot(:, i+1) = qdot(:, i) + qddot * dT;
    q(:, i+1) = q(:, i) + qdot(:, i) * dT + qddot * dT^2 / 2;
end
end

%%% Force-based Control %%%
if Mode == 3
Kd = 500 * eye(2);
Kp = 500 * eye(2);
alpha = 10;
Kqd = 1e-2 * eye(linkNum);
tau = zeros(linkNum, stepNum);
q(5,1) = 0.1;   % Set initial condition to avoid singularity
for i = 1:stepNum 
    M = CRBA(linkNum, Ii, h, q(:, i), linkLength);
    J = Jacobian_nR(linkNum, q(:, i), linkLength); 
    M_inv = eye(linkNum) / M;
    M_x = eye(2) / (J*M_inv*J');
    Jpinv_InertiaWeighted = M_inv * J' * M_x;
    
    J_dot = Jacobian_dot_nR(linkNum, q(:, i), qdot(:, i), linkLength);
    x_actual(:, i) = FK_nR(linkNum, q(:, i), linkLength); 
    x_errorNorm(i) = norm(x_actual(:, i) - x_desire(:,i));    
    xdot_actual    = J * qdot(:, i);
    xddot_ref      = xddot_desire(:, i) + Kd * (xdot_desire(:, i) - xdot_actual) + Kp * (x_desire(:, i) - x_actual(:, i));
    dg = q(:, i);
    null_tau = -Kqd * qdot(:, i) - alpha * dg;
    
    F = M_x * xddot_ref + M_x * (J * M_inv * ID(q(:, i), qdot(:, i), zeros(linkNum, 1), linkNum, h, Ii) - J_dot * qdot(:, i));
    %tau(:, i) = J' * F + (eye(linkNum) - J' * Jpinv_InertiaWeighted') * null_tau;
    %F = M_x * (xddot_ref - J_dot * qdot(:,i));
    tau(:, i) = ID(q(:, i), qdot(:, i), zeros(linkNum, 1), linkNum, h, Ii) + J' * F + (eye(linkNum) - J' * Jpinv_InertiaWeighted') * null_tau;
    qddot = M \ (tau(:, i) - ID(q(:,i), qdot(:,i), zeros(linkNum, 1), linkNum, h, Ii));
    qdot(:, i+1) = qdot(:, i) + qddot * dT;
    q(:, i+1) = q(:, i) + qdot(:, i) * dT + qddot * dT^2 / 2;
end
end

%%% Hierarchy Multitask Control %%%
if Mode == 4
Kp = 500 * eye(2);
q_lowerb = -ones(linkNum, 1);
q_upperb = ones(linkNum, 1);

x_center = [y_center; z_center];
xdot_center = zeros(2, 1);
for i = 2:stepNum
    J_tip = Jacobian_nR(linkNum, q(:, i), linkLength); 
    J_joint = Jacobian_nR(5, q(:, i), linkLength);
    x_actual(:, i) = FK_nR(linkNum, q(:, i), linkLength);  
    x_errorNorm(i) = norm(x_actual(:, i) - x_desire(:,i));    
    x_joint = FK_nR(5, q(:, i), linkLength);
    
    xdot_ref   = xdot_desire(:, i) + Kp * (x_desire(:,i) - x_actual(:, i));
    xdot_joint_ref = Kp * (x_center - x_joint);
    
    qdot_star1 = quadprog( blkdiag(zeros(linkNum, linkNum), eye(2)), zeros(linkNum+2, 1), [], [],...   
                           [J_tip, -eye(2)], xdot_ref);%, [q_lowerb - q(:,i-1); -Inf*ones(2,1)] / dT, [q_upperb - q(:,i-1); Inf*ones(2,1)] / dT);  
    W_star1 =  qdot_star1(end-1:end, 1);
    qdot_star2 = quadprog( blkdiag(zeros(linkNum, linkNum), eye(2)), zeros(linkNum+2, 1), [], [],...
                           [J_tip zeros(2,2); J_joint zeros(2,2) -eye(2, 2)], [xdot_ref + W_star1; xdot_joint_ref]);%,...
                           %[q_lowerb - q(:,i-1); -Inf*ones(2,1)] / dT, [q_upperb - q(:,i-1); Inf*ones(2,1)] / dT);
    
    qdot(:, i) = qdot_star2(1:linkNum, 1);
    q(:, i+1) = q(:, i) + qdot(:, i) * dT;
end
end

%%% Weighted Multitask Control %%%
if Mode == 5
Kp = 0.001 * eye(2);
W1 = 1e6;
W2 = 1e-2;
q_lowerb = -ones(linkNum, 1);
q_upperb = ones(linkNum, 1);

x_center = [y_center; z_center];
xdot_center = zeros(2, 1);
for i = 2:stepNum
    J_tip = Jacobian_nR(linkNum, q(:, i), linkLength); 
    J_joint = [Jacobian_nR(5, q(:, i), linkLength), zeros(2, 2)];
    x_actual(:, i) = FK_nR(linkNum, q(:, i), linkLength);  
    x_errorNorm(i) = norm(x_actual(:, i) - x_desire(:,i));    
    x_joint = FK_nR(5, q(:, i), linkLength);
    
    xdot_ref   = xdot_desire(:, i);
    xdot_joint_ref = Kp * (x_center - x_joint);
    
    H = 2 * W1 * (J_tip)' * J_tip + 2 * W2 * (J_joint)' * J_joint;
    f = -2 * (W1 * xdot_ref' * J_tip + W2 * xdot_joint_ref' * J_joint)';
    %qdot_star = quadprog(H, f, [], [], [], [], (q_lowerb - q(:,i)) / dT, (q_upperb - q(:,i)) / dT);  
    qdot_star = quadprog(H, f);%, dT*[eye(7); -eye(7)], [q_upperb - q(:,i); q(:,i) - q_lowerb]);  
    
    qdot(:, i) = qdot_star;
    q(:, i+1) = q(:, i) + qdot(:, i) * dT;
end
end

figure('position',[100 95 950 550])
plot(timeSeries, x_errorNorm, 'LineWidth', 1);
xlabel('time (s)','Fontname','Times', 'FontSize', 12);
ylabel('x_{error}','Fontname','Times', 'FontSize', 12);

trackingAnimation_nR(linkNum, q, linkLength, stepNum);

if Mode == 1 || Mode == 2 || Mode == 3 
    figure('position',[120 95 950 550])
    for i = 1:linkNum
        plot(timeSeries, tau(i,:), 'LineWidth', 1);
        hold on
    end
    xlabel('time (s)','Fontname','Times', 'FontSize', 12);
    ylabel('$\tau \; N{\cdot}m$', 'Interpreter', 'LaTex', 'FontSize',12);
    leng1 = legend('${\tau}_1$', '${\tau}_2$', '${\tau}_3$', '${\tau}_4$', '${\tau}_5$',...
                   '${\tau}_6$', '${\tau}_7$');
    set(leng1, 'Interpreter', 'LaTex', 'Location', 'northeast', 'Orientation', 'horizontal');            
end            
