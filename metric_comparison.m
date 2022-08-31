clear
close all
clc

%% Input

% anticlockwise
% DO_control_pts = [0.0, 0.2, 2.0,  3.6, 4.5, 4.35, 3.0, 1.75, 0.35, -0.20, 0;
%                   0.5, 0.1, 0.6, -0.1, 0.7, 1.60, 2.1, 2.20, 1.80, 1.30, 0.5] + [0.25; 0];
% for i = 1 : size(DO_control_pts, 2)
%     DO_control_pts(2, i) = 2.3 - DO_control_pts(2, i);
% end
% DO_control_pts = [0.02, 0.21, 2.0, 3.95, 4.65, 4.55, 3.0, 1.75, 0.35, -0.15, 0.02;
%                   0.5, 0.12, -0.25, -0.1, 0.7, 1.60, 2.1, 2.15, 1.80, 1.30, 0.5] + [0.25; 0];
DO_control_pts = [0.71, 1, 0.3,  2.5  4.7,  4, 4.7,  2.5,  0.3, 0.71;
                  1.64,  1, -0.4, 0,  -0.4, 1, 2.4,   2,    2.4, 1.64];

t = 0:0.01:1;
DO_contour_pts = open_quadratic_bspline(DO_control_pts, t);
DO_control_pts = 100 * DO_control_pts;
DO_contour_pts = 100 * DO_contour_pts;

% s_0 = [300, 340, 380, 420;
%        100, 100, 100, 100];
% e_i_direction = [-1, -1, -1, -1; 
%                  5, 5, 5, 5];       
% e_i_length = [30, 45, 60, 78];
% s_0 = [330, 370, 410;
%        155, 145, 125];
% e_i_direction = [-1, -1, -1; 
%                  -5, -5, -5];       
% e_i_length = [30, 45, 70];
s_0 = [280, 320, 360;
       170, 160, 150];
e_i_direction = [-1, -1, -1; 
                 -5, -5, -5];       
e_i_length = [60, 40, 30];


for col = 1 : size(e_i_direction, 2)
    e_i_direction(:, col) = e_i_direction(:, col) / norm(e_i_direction(:, col));
end
s_d = zeros(size(s_0));
for col = 1 : size(s_0, 2)
    s_d(:, col) = s_0(:, col) + e_i_length(1, col) * e_i_direction(:, col);
end
S_g = select_grasping_positions(DO_contour_pts, s_0, s_d);


%% figure
fig1 = figure('Position', [200 200 450 250]);
% h1 = subplot(1, 2, 1);
plot(DO_contour_pts(1, :), DO_contour_pts(2, :), 'Color', 'b', 'LineWidth', 1.5)
hold on
% plot(DO_control_pts(1, :), DO_control_pts(2, :), 'Color', 'r', 'LineWidth', 1.5)
% hold on
% plot([s_0(1, 1), s_0(1, 2)], [s_0(2,1), s_0(2, 2)], '-o', 'Color', 'k', 'LineWidth', 2, ...
%      'MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k')
% hold on
% plot([s_0(1, 1), s_0(1, 3)], [s_0(2,1), s_0(2, 3)], '-o', 'Color', 'k', 'LineWidth', 2, ...
%      'MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k') 
for i = 1 : size(s_0, 2)
    quiver(s_0(1, i), s_0(2, i), s_d(1, i) - s_0(1, i), s_d(2, i) - s_0(2, i), ...
           'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 1);
    hold on
end

plot(S_g(1, 1), S_g(2, 1), 'o', 'MarkerSize', 12, 'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'k')
hold on
plot(S_g(1, 2), S_g(2, 2), 's', 'MarkerSize', 12, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k')
hold on
plot(S_g(1, 3), S_g(2, 3), 'd', 'MarkerSize', 12, 'MarkerFaceColor', 'c', 'MarkerEdgeColor', 'k')
hold on
plot(S_g(1, 4), S_g(2, 4), '^', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k')
hold on
plot(S_g(1, 5), S_g(2, 5), 'p', 'MarkerSize', 12, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k')
hold on

% plot(s_g_2(1, 1), s_g_2(2, 1), 'o', 'MarkerSize', 12, 'MarkerFaceColor', 'g')
% hold on
% text(s_0(1,1) - 18, s_0(2,1) - 12, 's_1', 'FontSize', 20);
% text(s_0(1,2) + 10, s_0(2,2) - 10, 's_2', 'FontSize', 20);
% text(s_0(1,3) + 10, s_0(2,3) + 10, 's_3', 'FontSize', 20);
% text(s_g_1(1,1) - 15, s_g_1(2,1) - 18, 's_{g,1}', 'FontSize', 20);
% text(s_g_2(1,1) + 2, s_g_2(2,1) + 21, 's_{g,2}', 'FontSize', 20);
% text(28, 220, '(a)', 'FontName', 'Times', 'FontSize', 27);
% 
axis equal
% %axis equal
axis([0, 500, -50, 250])
% xlabel('x', 'Fontsize', 20)
% ylabel('y', 'Fontsize', 20)
grid on
%set(h1, 'Position', [0.08, 0.15, 0.4, 0.8])
% set(h1, 'FontSize', 18)[0.1300 0.1100 0.7750 0.8150]
set(gca, 'Position', [0.0500 0.1 0.94 0.88])
set(gca, 'YTick', -150:100:350)
set(gca, 'XTick', 0:100:500)
set(gca, 'FontSize', 12)
set(gcf, 'Renderer', 'Painters');
print(fig1, './Figure/Shape_3', '-depsc')


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


%%
function S_g = select_grasping_positions(DO_contour_pts, s_0, s_d) 
    S_g = zeros(2, 5);
    
    k = size(s_0, 2);
    
    e_i_vec = s_d - s_0;
    e_g = sum(e_i_vec, 2) / k;
    e_i_length = zeros(1, k);
    for i = 1 : k
        e_i_length(1, i) = norm(e_i_vec(:, i));
    end
  
    [~, original_s_idx] = sort(e_i_length, 'descend');
    max_e_i_idx = original_s_idx(1, 1);
    xi = 1.5 * e_i_length(max_e_i_idx) / norm(e_g);
    
    % max_e_i_length = e_i_length(1, max_e_i_idx);
    % e_i_weight = e_i_length / max_e_i_length;
    e_i_weight = e_i_length / sum(e_i_length, 2);

    s_0_centroid = sum(s_0, 2) / k;
    s_d_centroid = sum(s_d, 2) / k;

    s_0_weighted_centroid = zeros(2, 1);
    s_d_weighted_centroid = zeros(2, 1);
    for i = 1 : k
        s_0_weighted_centroid = s_0_weighted_centroid + e_i_weight(1, i) * s_0(:, i);
        s_d_weighted_centroid = s_d_weighted_centroid + e_i_weight(1, i) * s_d(:, i);
    end
    
    % [M_CM_max, M_DWCM_max, M_PCM_max, M_DWPCM_max, M_TOM_max]
    metrics_max = zeros(1, 5);
    for i = 1 : size(DO_contour_pts, 2)
        s_g = DO_contour_pts(:, i);
        s_d_g = s_g + xi * e_g;
        metrics = zeros(1, 5);
        
        for j = 1 : k
            s_0_i = s_0(:, j);
            M_j = compute_static_metric(s_g, s_0_i, s_d(:, j));
            metrics(1, 1) = metrics(1, 1) + M_j;
            metrics(1, 2) = metrics(1, 2) + e_i_weight(1, j) * M_j;
            metrics(1, 5) = metrics(1, 5) + integrate_metric(s_g, s_d_g, s_0_i, s_d(:, j));
        end
        metrics(1, 3) = compute_static_metric(s_g, s_0_centroid, s_d_centroid);
        metrics(1, 4) = compute_static_metric(s_g, s_0_weighted_centroid, s_d_weighted_centroid);
        for idx = 1 : 5
            if metrics(1, idx) > metrics_max(1, idx)
                metrics_max(1, idx) = metrics(1, idx);
                S_g(:, idx) = s_g;
            end
        end
    end
end

%%
function M_i = integrate_metric(s_g, s_d_g, s_0, s_d)
    M_i = 0;
    
    e_g = s_d_g - s_g;
    e_i = s_d - s_0;
    steps = 100;
    delta_path = norm(e_i) / steps;
    for i = 1 : steps - 1
        s_g_cur = s_g + i / steps * e_g;
        s_i_cur = s_0 + i / steps * e_i;
        M_i_cur = compute_static_metric(s_g_cur, s_i_cur, s_d);
        M_i = M_i + M_i_cur * delta_path;
    end
end


%%
function M_i = compute_static_metric(s_g, s_0, s_d) 
        k_d = 0.002;
        gamma = 0.4;
        r = 10;
        delta_r = 2;
        
        d_i = s_0 - s_g;
        d_i_norm = norm(d_i);
        e_i = s_d - s_0;
        cos_val = e_i' * d_i / (norm(e_i) * norm(d_i));
        if cos_val < -1
            cos_val = -1;
        elseif cos_val > 1
            cos_val = 1;
        end
        alpha_i = acos(cos_val);

        h_1 = exp(-k_d * d_i_norm);
        h_2 = 1 - gamma * sin(alpha_i);
        h_3 = 1;
        if d_i_norm <= r
            h_3 = 1 / h_2;
        elseif d_i_norm <= r + delta_r
            h_3 = (1 - (d_i_norm - r) / delta_r * gamma * sin(alpha_i)) / h_2;
        end
        M_i = h_1 * h_2 * h_3;    
end
