clear
close all
clc

%% Input
% clockwise
% DO_control_pts = [0.0, 0.2, 2.0,  3.6, 4.5, 4.35, 3.0, 1.75, 0.35, -0.20, 0;
%                   0.5, 0.1, 0.6, -0.1, 0.7, 1.60, 2.1, 2.20, 1.80, 1.30, 0.5] + [0.25; 0];
DO_control_pts = [0.20, 0.72, 2.20, 3.70, 5.16, 5.27, 4.74, 3.70, 2.00, -0.25, 0.20;
              0.50, 0.00, 1.20, 0.15, 0.70, 1.40, 1.95, 1.50, 2.50, 1.20, 0.50];
DO_control_pts(1,:) = DO_control_pts(1,:) * 0.95;
DO_control_pts(2,:) = DO_control_pts(2,:) * 1.05;              
t = 0:0.02:1;
DO_contour_pts = open_quadratic_bspline(DO_control_pts, t);
DO_control_pts = 100 * DO_control_pts;
DO_contour_pts = 100 * DO_contour_pts;

s_0 = [100, 400, 200;
       110, 120, 175];
s_0_opp = s_0;
e_i_direction = [-1, 1, 0;
                 0, 0, -1];       
e_i_direction_opp = e_i_direction;
e_i_length = [50, 50, 25];
for col = 1 : size(e_i_direction, 2)
    e_i_direction(:, col) = e_i_direction(:, col) / norm(e_i_direction(:, col));
    e_i_direction_opp(:, col) = e_i_direction_opp(:, col) / norm(e_i_direction_opp(:, col));
end
s_d = zeros(size(s_0));
s_d_opp = zeros(size(s_0_opp));
for col = 1 : size(s_0, 2)
    s_d(:, col) = s_0(:, col) + e_i_length(1, col) * e_i_direction(:, col);
    s_d_opp(:, col) = s_0_opp(:, col) + e_i_length(1, col) * e_i_direction_opp(:, col);
end
[norm_value, original_s_idx] = sort(e_i_length, 'descend');
idx_1 = original_s_idx(1, 1);
idx_2 = original_s_idx(1, 2);
s_g_1 = get_grasping_position_for_single_pt(DO_contour_pts, s_0(:, idx_1), s_d(:, idx_1), 0.02);
s_g_2 = get_grasping_position_for_single_pt(DO_contour_pts, s_0(:, idx_2), s_d(:, idx_2), 0.02);
s_g_1_opp = get_grasping_position_for_single_pt(DO_contour_pts, s_0_opp(:, idx_1), s_d_opp(:, idx_1), 0.005);
s_g_2_opp = get_grasping_position_for_single_pt(DO_contour_pts, s_0_opp(:, idx_2), s_d_opp(:, idx_2), 0.005);

%% figure
fig = figure('Position', [200 200 1200 400]);
h1 = subplot(1, 2, 1);
plot(DO_contour_pts(1, :), DO_contour_pts(2, :), 'Color', 'b', 'LineWidth', 1.5)
hold on
plot(s_0(1, 1), s_0(2, 1), '-o', 'Color', 'k', 'LineWidth', 2, ...
     'MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k')
hold on
plot(s_0(1, 2), s_0(2, 2), '-o', 'Color', 'k', 'LineWidth', 2, ...
     'MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k')
 hold on
plot(s_0(1, 3), s_0(2, 3), '-o', 'Color', 'k', 'LineWidth', 2, ...
     'MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k')
for i = 1 : size(s_0, 2)
    quiver(s_0(1, i), s_0(2, i), s_d(1, i) - s_0(1, i), s_d(2, i) - s_0(2, i), ...
           'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 1);
    hold on
end
plot(s_g_1(1, 1), s_g_1(2, 1), 'o', 'MarkerSize', 12, 'MarkerFaceColor', 'r')
hold on
plot(s_g_2(1, 1), s_g_2(2, 1), 'o', 'MarkerSize', 12, 'MarkerFaceColor', 'g')
hold on
text(s_0(1,1) - 12, s_0(2,1) - 15, 's_1', 'FontSize', 18);
text(s_0(1,2) -12, s_0(2,2) - 15, 's_2', 'FontSize', 18);
text(s_0(1,3) + 10, s_0(2,3) + 0, 's_3', 'FontSize', 18);
text(s_g_1(1,1) - 10, s_g_1(2,1) - 24, 's_{g,1}', 'FontSize', 18);
text(s_g_2(1,1) - 10, s_g_2(2,1) + 32, 's_{g,2}', 'FontSize', 18);
axis equal
%axis equal
axis([0, 500, -50, 250])
xlabel('x', 'Fontsize', 18)
ylabel('y', 'Fontsize', 18)
grid on
set(h1, 'Position', [0.08, 0.15, 0.4, 0.8])
set(h1, 'FontSize', 18)
set(h1, 'YTick', -150:100:350)


h2 = subplot(1, 2, 2);
plot(DO_contour_pts(1, :), DO_contour_pts(2, :), 'Color', 'b', 'LineWidth', 1.5)
hold on
plot(s_0_opp(1, 1), s_0_opp(2, 1), '-o', 'Color', 'k', 'LineWidth', 2, ...
     'MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k')
hold on
plot(s_0_opp(1, 2), s_0_opp(2, 2), '-o', 'Color', 'k', 'LineWidth', 2, ...
     'MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k')
 hold on
plot(s_0_opp(1, 3), s_0_opp(2, 3), '-o', 'Color', 'k', 'LineWidth', 2, ...
     'MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k')
for i = 1 : size(s_0, 2)
    quiver(s_0_opp(1, i), s_0_opp(2, i), s_d_opp(1, i) - s_0_opp(1, i), s_d_opp(2, i) - s_0_opp(2, i), ...
           'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 1);
    hold on
end
plot(s_g_1_opp(1, 1), s_g_1_opp(2, 1), 'o', 'MarkerSize', 12, 'MarkerFaceColor', 'r')
hold on
plot(s_g_2_opp(1, 1), s_g_2_opp(2, 1), 'o', 'MarkerSize', 12, 'MarkerFaceColor', 'g')
hold on
text(s_0_opp(1,1) - 12, s_0_opp(2,1) - 15, 's_1', 'FontSize', 18);
text(s_0_opp(1,2) - 12, s_0_opp(2,2) - 15, 's_2', 'FontSize', 18);
text(s_0_opp(1,3) + 10, s_0_opp(2,3), 's_3', 'FontSize', 18);
text(s_g_1_opp(1,1) - 10, s_g_1_opp(2,1) - 24, 's_{g,1}', 'FontSize', 18);
text(s_g_2_opp(1,1) - 48, s_g_2_opp(2,1) + 32, 's_{g,2}', 'FontSize', 18);
axis equal
%axis equal
axis([0, 500, -50, 250])
xlabel('x', 'Fontsize', 18)
ylabel('y', 'Fontsize', 18)
grid on
set(h2, 'Position', [0.56, 0.15, 0.4, 0.8])
set(h2, 'FontSize', 18)
set(h2, 'YTick', -150:100:350)
set(gcf, 'Renderer', 'Painters');
print(fig, './Figure/3pts_feature', '-depsc')

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
for i = 2 : n - 3
    B = P(:, i : i + 2) * M2 * T;
    S = [S B];
end
% calculate the last Bezier curve
B = P(:, n - 2 : n) * M3 * T;
S = [S B];
end

%% Discretize the DO contour
function s_g = get_grasping_position_for_single_pt(DO_contour_pts, s_0, s_d, k)
    Q_contour = zeros(1, size(DO_contour_pts, 2));
    Q_max_value = 0;
    % k = 0.02;
    gamma = 0.2;
    r = 5;
    delta_r = 10;
    delta_p = 0.1;
    delta_g = 0.2;
    alpha_0 = pi / 6;
    alpha_1 = pi / 6 * 5;
    for contour_idx = 1 : size(DO_contour_pts, 2)
            Q = 0;
            for s_idx = 1 : size(s_0, 2)
                s_g = DO_contour_pts(:, contour_idx);
                s_0_i = s_0(:, s_idx);
                s_d_i = s_d(:, s_idx);
                e_i = s_d_i - s_0_i;
                e_i_norm = norm(e_i);            
                d_i = s_0_i - s_g;
                d_i_norm = norm(d_i);
                alpha_i = acos(e_i' * d_i / (e_i_norm * d_i_norm));
                xi = 1 + delta_d;
                if alpha_i <= alpha_0 || alpha_i >= alpha_1
                    xi = 1 + delta_p;
                end
                s_d_g = s_g + xi * e_i;

                % integrate
                step_num = 100;
                step_len = e_i_norm / step_num;
                q_i = 0;
                for i = 1 : step_num
                    s_g_cur = s_g + i / step_num * xi * e_i;
                    s_0_cur = s_0_i + i / step_num * e_i;
                    d_i = s_0_cur - s_g_cur;
                    d_i_norm = norm(d_i);
                    % d_i_norm = sqrt(d_i_norm);
                    alpha_i = acos(e_i' * d_i / (e_i_norm * d_i_norm));   
                    lambda = 1;
                    if d_i_norm <= r
                        lambda = 1 / (1 - gamma * sin(alpha_i));
                    end
                    if d_i_norm > r && d_i_norm < r + delta_r
                        lambda = 1 / (1 - gamma * sin(alpha_i)) * (1 - (d_i_norm - r) / delta_r * gamma * sin(alpha_i));
                    end         
                    M = exp(-k * d_i_norm) * (1 - gamma * sin(alpha_i)) * lambda;
                    q_i = q_i + M * step_len;
                end
                Q = Q + q_i;
            end
            Q_contour(1, contour_idx) = Q;
            if Q > Q_max_value
                Q_max_value = Q;
            end
    end
    Q_contour = Q_contour / Q_max_value;
    [~, Q_max_idx] = max(Q_contour);
    s_g = DO_contour_pts(:, Q_max_idx);
end
