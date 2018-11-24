%%% ENGG5402 HW4_2b, Forward Dynamics
%%% The ODE is: M(q, qdot)*qddot + C(q, qdot)*qdot + g(q) = tau

clear
close all
clc

q0    = [0; pi/2];
tau   = [0; 0];
T     = 5;
dt    = 0.01;
t     = 0:dt:T;
steps = length(t);

q     = zeros(2, steps);
qdot  = zeros(2, steps);
q(:,1) = q0;

for i = 1:steps - 1
    q_1    = q(1, i);
    q_2    = q(2, i);
    qdot_1 = qdot(1, i);
    qdot_2 = qdot(2, i);
    s2     = sin(q_2);
    c2     = cos(q_2);
    
    M = [10 + s2^2    c2;
         c2           2 ];
    C = [s2*c2*qdot_2   s2*c2*qdot_1 - s2*qdot_2;
         -s2*c2*qdot_1  0 ];
    g = [0; 10*s2];
    qddot = M \ (tau - C*qdot(:,i) - g);
    qdot(:, i+1) = qdot(:, i) + qddot * dt;
    q(:,i+1) = q(:, i) + qdot(:, i) * dt;
end

figure('position', [100 60 1000 580]);
H11 = subplot(2,2,1);
plot(t, q(1,:), 'Linewidth', 1.5)
ylabel('$\theta_1$','Interpreter','LaTex','Fontsize', 12);
set(gca,'XTick',[]); 
%title('$Cubic \; Polynomial \; Trajectory$','Interpreter','LaTex','Fontsize', 12);

H21 = subplot(2,2,3);
plot(t, qdot(1,:), 'Linewidth', 1.5)
ylabel('$\dot{\theta_1}$','Interpreter','LaTex','Fontsize', 12);
xlabel('\it{Time (s)}','Fontname','Times','Fontsize',12);

H12 = subplot(2,2,2);
plot(t, q(2,:), 'Linewidth', 1.5);
ylabel('$\theta_2$','Interpreter','LaTex','Fontsize',12);
%xlabel('\it{Time (s)}','Fontname','Times','Fontsize',12);
set(gca, 'XTick',[]);

H22 = subplot(2,2,4);
plot(t, qdot(2,:), 'Linewidth', 1.5);
ylabel('$\dot{\theta_2}$', 'Interpreter', 'LaTex', 'Fontsize', 12);
xlabel('\it{Time (s)}', 'Fontname', 'Times', 'Fontsize', 12);


%%% Use ODE45 to solve the ODE. Pay special attention to the function used
%%% in this solver. It showns even the ODE cannot be explicitly
%%% expressed by linear equation set. As long as it can be computed
%%% explicitly, this solver can still be applied, though some tricks are
%%% needed to get the corresponding ode.

tspan = [0 5];
x_init = [0; 0; pi/2; 0];
[t1, x] = ode45(@ARHW4_2ii, tspan, x_init);


figure('position', [100 60 1000 580]);
H11_2 = subplot(2,2,1);
plot(t, q(1,:), 'Linewidth', 1.5)
hold on
plot(t1, x(:,1), 'Color', 'r', 'Linewidth', 1.5);
ylabel('$\theta_1$','Interpreter','LaTex','Fontsize', 12);
set(gca,'XTick',[]); 
leng1 = legend('Euler integration', 'ODE45');
set(leng1, 'Box', 'Off');
%title('$Cubic \; Polynomial \; Trajectory$','Interpreter','LaTex','Fontsize', 12);

H21 = subplot(2,2,3);
plot(t, qdot(1,:), 'Linewidth', 1.5)
hold on
plot(t1, x(:,2), 'Color', 'r', 'Linewidth', 1.5);
ylabel('$\dot{\theta_1}$','Interpreter','LaTex','Fontsize', 12);
xlabel('\it{Time (s)}','Fontname','Times','Fontsize',12);
leng2 = legend('Euler integration', 'ODE45');
set(leng2, 'Box', 'Off');

H12 = subplot(2,2,2);
plot(t, q(2,:), 'Linewidth', 1.5);
hold on
plot(t1, x(:,3), 'Color', 'r', 'Linewidth', 1.5);
ylabel('$\theta_2$','Interpreter','LaTex','Fontsize',12);
%xlabel('\it{Time (s)}','Fontname','Times','Fontsize',12);
set(gca, 'XTick',[]);
leng3 = legend('Euler integration', 'ODE45');
set(leng3, 'Box', 'Off');

H22 = subplot(2,2,4);
plot(t, qdot(2,:), 'Linewidth', 1.5);
hold on
plot(t1, x(:,4), 'Color', 'r', 'Linewidth', 1.5);
ylabel('$\dot{\theta_2}$', 'Interpreter', 'LaTex', 'Fontsize', 12);
xlabel('\it{Time (s)}', 'Fontname', 'Times', 'Fontsize', 12);
leng4 = legend('Euler integration', 'ODE45');
set(leng4, 'Box', 'Off');