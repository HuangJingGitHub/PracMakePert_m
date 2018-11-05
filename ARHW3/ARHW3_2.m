%%% ENGG5402 HW3_2c&d Individual Joint Control: Optimal
%%% Transmission/Impedance Matching

clear
close all;

%%% 2(c) 
dr = 5/100;
r_n = 0:dr:5;
a_n = 2*r_n./(r_n.^2 + 1);

figure('Position',[100 80 850 500])
plot(r_n, a_n, 'LineWidth', 2);
xlabel('$Normialized \; Transmission \;Ratio \; r^*$', 'Interpreter', 'LaTex', 'Fontsize', 12);
ylabel('$Normialized \; Acceleration \; a^*$', 'Interpreter', 'LaTex', 'Fontsize', 12);
axis([0 5 0 1.2]);
hold on
line([0 1], [1 1], 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
line([1 1], [0 1], 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
line([1 1], [1 1], 'Color', 'r', 'Marker', '.', 'MarkerSize', 25);

%%% 2(d)
k = [1.25 1.5 1.75];
a_nk(1,:) = 2*r_n./(k(1)*r_n.^2 + 1);
a_nk(2,:) = 2*r_n./(k(2)*r_n.^2 + 1);
a_nk(3,:) = 2*r_n./(k(3)*r_n.^2 + 1);

figure('Position',[100 80 850 500])
plot(r_n, a_nk(1,:), 'LineWidth', 2);
hold on
plot(r_n, a_nk(2,:), 'Color', 'b', 'LineWidth', 2);
hold on
plot(r_n, a_nk(3,:), 'Color', 'g', 'LineWidth', 2);
xlabel('$Normialized \; Transmission \;Ratio \; r^*$', 'Interpreter', 'LaTex', 'Fontsize', 12);
ylabel('$Normialized \; Acceleration \; a^*$', 'Interpreter', 'LaTex', 'Fontsize', 12);
axis([0 5 0 1.2]);

hold on
tp = sqrt(1./k);
t1 = tp(1);
t2 = tp(2);
t3 = tp(3);
line([0 t1], [t1 t1], 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1);
line([t1 t1], [0 t1], 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1);
line([t1 t1],[t1 t1], 'Color', 'r', 'Marker', '.', 'MarkerSize', 25);

line([0 t2], [t2 t2], 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1);
line([t2 t2], [0 t2], 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1);
line([t2 t2],[t2 t2], 'Color', 'r', 'Marker', '.', 'MarkerSize', 25);

line([0 t3], [t3 t3], 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1);
line([t3 t3], [0 t3], 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1);
line([t3 t3],[t3 t3], 'Color', 'r', 'Marker', '.', 'MarkerSize', 25);
legd = legend('k = 1.25', 'k = 1.5', 'k = 1.75');
set(legd,'Box','off');
