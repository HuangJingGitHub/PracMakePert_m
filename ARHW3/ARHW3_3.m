%%% ENGG5402 HW3_3(b)&(c)

clear
close all

tspan = [0 5];
x_init = [0; 0];
[t, x] = ode45(@ARHW3_3b, tspan, x_init);

figure('Position', [100, 80, 900, 500])
plot(t,x(:,1), 'Color', 'r', 'LineWidth', 1.5)
axis([0 5 0 0.055])
xlabel('$Time\;(s)$','Interpreter', 'LaTex', 'Fontsize', 12);
ylabel('$\theta$', 'Interpreter', 'LaTex', 'Fontsize', 12);
title('$Step\;Response$', 'Interpreter', 'LaTex', 'FontSize', 12);

[t1, x1] = ode45(@ARHW3_3c, tspan, x_init);
figure('Position', [100, 80, 900, 500])
plot(t1,x1(:,1), 'Color', 'r', 'LineWidth', 1.5)
axis([0 5 0 0.006])
xlabel('$Time\;(s)$','Interpreter', 'LaTex', 'Fontsize', 12);
ylabel('$\theta$', 'Interpreter', 'LaTex', 'Fontsize', 12);
title('$Response \; with \; \tau_d = 0.1 \; N{\cdot}m$', 'Interpreter', 'LaTex', 'FontSize', 12);