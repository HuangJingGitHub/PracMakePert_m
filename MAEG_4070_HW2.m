clear
close all
clc

x_11 = -4:0.1:0.8;
x_12 = 0.8:0.1:6;
f_11 = x_11.^2 + 4;
f_12 = x_12.^2 + 4;
f_21 = (x_11 - 2).^2 + 3;
f_22 = (x_12 - 2).^2 + 3;
center = [1, 5];
parameterAng = 0:2*pi/100:2*pi;
r = 0.5;
circle_x_1 = center(1) + r*cos(parameterAng);
circle_x_2 = center(2) + r*sin(parameterAng);

figure('Position',[100 80 850 500])
p1 = plot(x_11, f_11, '-b', 'LineWidth', 2);
hold on
plot(x_12, f_12, '--b', 'LineWidth', 2)
hold on
plot(x_11, f_21, '--r', 'LineWidth', 2)
hold on
p2 = plot(x_12, f_22, '-r', 'LineWidth', 2);
hold on
plot(center(1), center(2), 'Marker', '.', 'MarkerSize', 25, 'Color', 'k')
hold on
plot(circle_x_1, circle_x_2, '--k','LineWidth', 2)
axis([-3 5 2.5 8]);
axis equal
xlabel('x_1','Fontname','Times', 'FontSize', 12);
ylabel('x_2','Fontname','Times', 'FontSize', 12);
legend([p1, p2], 'x_2 = x_1^2 + 4', 'x_2 = (x_1 - 2)^2 + 3')