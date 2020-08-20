close all
clear
clc

% 2D control points
P = [0.0, 0.2, 2.0,  3.6, 4.5, 4.35, 3.0, 1.75, 0.35, -0.20, 0;
     0.5, 0.1, 0.6, -0.1, 0.7, 1.60, 2.1, 2.20, 1.80, 1.30, 0.5];
t = 0:0.02:1;
C = open_quadratic_bspline(P,t);

%% simulation setting
% sacle to image space
P = 100 * P;
C = 100 * C;
s = [150, 300, 275;
     110, 110, 175];
p_length = (max(C(1,:)) - min(C(1,:))) / 5;

%% figure
fig1 = figure('position', [100 50 800 600]);
% plot(P(1,:), P(2,:), '-o','MarkerSize', 5, 'MarkerEdgeColor','r', ...
%     'MarkerFaceColor','r')
% hold on
plot([s(1,1), s(1,2)], [s(2,1), s(2,2)], '-o', 'Color', 'k', 'Linewidth', 2, ...
    'MarkerSize', 5, 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
text(s(1,1)-15, s(2,1)-10, 's_0', 'FontSize', 13);
text(s(1,2)+10, s(2,2)-10, 's_1', 'FontSize', 13);
text(s(1,3)+10, s(2,3)-10, 's_2', 'FontSize', 13);

hold on
plot([s(1,1), s(1,3)], [s(2,1), s(2,3)], '-o', 'Color', 'g', 'LineWidth', 2, ...
    'MarkerSize', 5, 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
hold on
plot(C(1,:), C(2,:), 'Color', 'b', 'LineWidth', 1.5)
hold on 
p_line = line([0, 50], [0, 50], 'Marker', 's', 'Color', 'r', 'Linewidth', 2, ...
        'MarkerSize', 5, 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
textSl = text(0, 0, 's_{el}', 'FontSize', 13);
textSr = text(0, 0, 's_{er}', 'FontSize', 13);
textNe = text(0, 0, 'n_e', 'FontSize', 13);
neArrow = quiver(1,0,30,30, 'LineWidth',2, 'Color', 'r', 'MaxHeadSize', 4);
axis equal
axis([-20, 450, -50, 250])
grid on
xlabel('x (px)', 'Fontsize', 20)
ylabel('y (px)', 'Fontsize', 20)

%% calculate TCI in loop in first 3 curves
steps = 110;
TCI = zeros(1, steps);
p_midX = zeros(1, steps);
for i = 1:steps
    s_el = C(:, i);
    [s_er, TCI(1, i)] = calcTCI(C, i, p_length, s);
    s_em = (s_el + s_er) / 2;
    p_midX(1, i) = s_em(1,1);
    p_direction = (s_er - s_el) / norm(s_er - s_el);
    ne_direction = [-p_direction(2,1); p_direction(1,1)];
    
    % plot update
    set(p_line,'xdata',[s_el(1,1) s_er(1,1)], 'ydata',[s_el(2,1) s_er(2,1)]);
    set(textSl, 'Position', s_el + [-15; -10]);
    set(textSr, 'Position', s_er + [-15; -10]);
    set(textNe, 'Position', s_em + [10; 15]);
    set(neArrow, 'XData', s_em(1,1), 'YData', s_em(2,1), 'UData', 36*ne_direction(1,1), 'VData', 36*ne_direction(2,1));
    % plot([i-1, i], [TCI(1,i-1), TCI(1,i)]*1e7, '-', 'Color', 'b', 'LineWidth', 1.5)
    hold on
    pause(0.05);
end
fig2 = figure('position', [200 50 800 600]);
plot(p_midX, TCI*1e7, 'Color', 'm', 'LineWidth', 2)
xlabel('x (px)', 'Fontsize', 20)
ylabel('TCI', 'Fontsize', 20)
grid on

print(fig1, './figure/Contour_Setting', '-depsc')
print(fig2, './figure/TMI', '-depsc')
%% function from MMU-UK for open uniform B-spline curves
function S = open_quadratic_bspline(P, t)
M1 = 1/2 * [2 -4 2; -3 4 0; 1 0 0];
M2 = 1/2 * [1 -2 1; -2 2 1; 1 0 0];
M3 = 1/2 * [1 -2 1; -3 2 1; 2 0 0];
T = [t.^2; t; t.^0];

% get n
n = size(P, 2);

% calculate the first Bezier curve
B = P(:, 1:3) * M1 * T;
S = B;
% middle curves
for i = 2 : n-3
    B = P(:, i:i+2) * M2 * T;
    S = [S B];
end
% calculate the last Bezier curve
B = P(:, n-2:n) * M3 * T;
S = [S B];

end


%% calculate task-oriented contact index (TCI)
function [s_r, TCI] = calcTCI(C, selIdx, p_length, s)
    % find s_er
    s_l = C(:, selIdx);
    s_r = [];
    thresholdLen = p_length / 24;
    for i = selIdx : 2 : size(C,2)
        s_r = C(:, i);
        if (abs(norm(s_r - s_l) - p_length) <= thresholdLen)
            break
        end
    end
    
    % angle features
    Eindictor = zeros(3,1); % 0-El, 1-E, 2-Er
    distances = zeros(3,1);
    W = zeros(6,1);
    s_0 = s(:, 1);
    s_1 = s(:, 2);
    s_2 = s(:, 3);
    x_0 = s_0(1,1); y_0 = s_0(2,1);
    x_1 = s_1(1,1); y_1 = s_1(2,1);
    x_2 = s_2(1,1); y_2 = s_2(2,1);
    
    v_1 = s_1 - s_0;
    v_2 = s_2 - s_0;
    
    t_0 = v_1' * v_2;
    vn_1 = norm(v_1);
    vn_2 = norm(v_2);
    t_1 = t_0 / vn_1 / vn_2;
    y = acos(t_1);
    Jy = zeros(1, 6);
    
    Jy(1,1) = (-1/sqrt(1 - t_1^2)) * ((2*x_0 - x_1 - x_2) * vn_1^2 * vn_2^2 - t_0 * ...
                ((x_0 - x_1) * vn_2^2 + (x_0 - x_2) * vn_1^2)) / vn_1^3 / vn_2^3;
    Jy(1,2) = (-1/sqrt(1 - t_1^2)) * ((2*y_0 - y_1 - y_2) * vn_1^2 * vn_2^2 - t_0 * ...
                ((y_0 - y_1) * vn_2^2 + (y_0 - y_2) * vn_1^2)) / vn_1^3 / vn_2^3;
    Jy(1,3) = (-1/sqrt(1 - t_1^2)) * ((x_2 - x_0) * vn_1^2 * vn_2 - (x_1 - x_0) * t_0 * vn_2)...
                / vn_1^3 / vn_2^2;
    Jy(1,4) = (-1/sqrt(1 - t_1^2)) * ((y_2 - y_0) * vn_1^2 * vn_2 - (y_1 - y_0) * t_0 * vn_2)...
                / vn_1^3 / vn_2^2;
    Jy(1,5) = (-1/sqrt(1 - t_1^2)) * ((x_1 - x_0) * vn_1 * vn_2^2 - (x_2 - x_0) * t_0 * vn_1)...
                / vn_1^2 / vn_2^3;
    Jy(1,6) = (-1/sqrt(1 - t_1^2)) * ((y_1 - y_0) * vn_1 * vn_2^2 - (y_2 - y_0) * t_0 * vn_1)...
                / vn_1^2 / vn_2^3;
    
    % p plane related parameters
    a = s_r(1,1) - s_l(1,1);
    b = s_r(2,1) - s_l(2,1);
    cl = -a * s_l(1,1) - b * s_l(2,1);
    cr = -a * s_r(1,1) - b * s_r(2,1);
    s_em = (s_l + s_r) / 2;
    refl = a*s_em(1,1) + b*s_em(2,1) + cl;
    refr = a*s_em(1,1) + b*s_em(2,1) + cr;
    divCof = 1 / norm(s_r - s_l);
    
    for i = 1:3
        linel = a * s(1,i) + b * s(2,i) + cl;
        liner = a * s(1,i) + b * s(2,i) + cr;
        if (refl * linel >= 0 && refr * liner >= 0)
            Eindictor(i,1) = 1;
            distances(i,1) = divCof * abs(b * s(1,i) - a * s(2,i) - b * s(1,i) + a * s(2,i));
        elseif(refl * linel < 0)
            Eindictor(i,1) = 0;
            distances(i,1) = divCof * abs(linel);
        else
            Eindictor(i,1) = 2;
            distances(i,1) = divCof * abs(liner);
        end    
    end
    epsilonE = 10;
    epsilonp = 100;
    
    for i = 1:3
        if (Eindictor(i,1) == 1)
           W(2*i-1, 1) = 1 / (distances(i, 1) + epsilonE);
           W(2*i, 1) = W(2*i-1,1);
        else
           W(2*i-1, 1) = 1 / (distances(i, 1) + epsilonp);
           W(2*i, 1) = W(2*i-1, 1);
        end
    end
    M = abs(Jy) * W;
    TCI = norm(M)^2;
end