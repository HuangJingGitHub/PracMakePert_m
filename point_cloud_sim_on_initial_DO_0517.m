clear
close all
clc

%% Initialzie 
[xy_x, xy_y] = meshgrid(0.5 : 2 : 100, 0.5 : 2 : 50);
xy_z = 20 * ones(size(xy_x));
[xz_x, xz_z] = meshgrid(0.5 : 2 : 100, 0.5 : 2 : 20);
xz_y = zeros(size(xz_x));
[yz_y, yz_z] = meshgrid(0.5 : 2 : 50, 0.5 : 2 : 20);
yz_x = zeros(size(yz_y));

%% 
k_x = 1 / 50000;
l_x = 100;
k_y = 1 / 25000;
l_y = 50;
xy_z_deformed = xy_z;
% xy_z_deformed_retraction = xy_z;
for row = 1 : size(xy_z, 1)
    for col = 1 : size(xy_z, 2)
        temp_x = xy_x(row, col);
        xy_z_deformed(row, col) = xy_z(row, col) + k_x * temp_x * temp_x * (3 * l_x - temp_x);
        temp_y = xy_y(row, col);
        xy_z_deformed(row, col) = xy_z_deformed(row, col) + k_y * temp_y * temp_y * (3 * l_y - temp_y);
    end
end
xz_z_deformed = xz_z;
%xz_z_deformed_retraction = xz_z;
for row = 1 : size(xz_z, 1)
    for col = 1 : size(xz_z, 2)
        temp_x = xz_x(row, col);
        xz_z_deformed(row, col) = xz_z(row, col) + k_x * temp_x * temp_x * (3 * l_x - temp_x);
        temp_y = xz_y(row, col);
        xz_z_deformed(row, col) = xz_z_deformed(row, col) + k_y * temp_y * temp_y * (3 * l_y - temp_y);
    end
end
yz_z_deformed = yz_z;
for row = 1 : size(yz_z, 1)
    for col = 1 : size(yz_z, 2)
        temp_y = yz_y(row, col);
        yz_z_deformed(row, col) = yz_z(row, col) + k_y * temp_y * temp_y * (3 * l_y - temp_y);
    end
end

%% Find boundaries
yz_boundary = zeros(3, 2 * size(yz_x, 1) + 2 * size(yz_x, 2) - 4);
yz_boundary_deformed = yz_boundary;
cnt = 1;
for col = 1 : size(yz_x, 2)
    yz_boundary(:, cnt) = [yz_x(1, col); yz_y(1, col); yz_z(1, col)];
    yz_boundary_deformed(:, cnt) = [yz_x(1, col); yz_y(1, col); yz_z_deformed(1, col)];
    cnt = cnt + 1;
end
for row = 2 : size(yz_x, 1)
    yz_boundary(:, cnt) = [yz_x(row, end); yz_y(row, end); yz_z(row, end)];
    yz_boundary_deformed(:, cnt) = [yz_x(row, end); yz_y(row, end); yz_z_deformed(row, end)];
    cnt = cnt + 1;
end
for col = size(yz_x, 2) - 1 : -1 : 1
    yz_boundary(:, cnt) = [yz_x(end, col); yz_y(end, col); yz_z(end, col)];
    yz_boundary_deformed(:, cnt) = [yz_x(end, col); yz_y(end, col); yz_z_deformed(end, col)];
    cnt = cnt + 1;
end
for row = size(yz_x, 1) - 1 : -1 : 2
    yz_boundary(:, cnt) = [yz_x(row, 1); yz_y(row, 1); yz_z(row, 1)];
    yz_boundary_deformed(:, cnt) = [yz_x(row, 1); yz_y(row, 1); yz_z_deformed(row, 1)];
    cnt = cnt + 1;
end

xy_boundary = zeros(3, size(xy_x, 1) + 2 * size(xy_x, 2) - 4);
xy_boundary_deformed = xy_boundary;
cnt = 1;
for col = 2 : size(xy_x, 2)
    xy_boundary(:, cnt) = [xy_x(1, col); xy_y(1, col); xy_z(1, col)];
    xy_boundary_deformed(:, cnt) = [xy_x(1, col); xy_y(1, col); xy_z_deformed(1, col)];
    cnt = cnt + 1;
end
for row = 2 : size(xy_x, 1)
    xy_boundary(:, cnt) = [xy_x(row, end); xy_y(row, end); xy_z(row, end)];
    xy_boundary_deformed(:, cnt)= [xy_x(row, end); xy_y(row, end); xy_z_deformed(row, end)];
    cnt = cnt + 1;
end
for col = size(xy_x, 2) - 1 : - 1 : 2
    xy_boundary(:, cnt) = [xy_x(end, col); xy_y(end, col); xy_z(end, col)];
    xy_boundary_deformed(:, cnt) = [xy_x(end, col); xy_y(end, col); xy_z_deformed(end, col)];
    cnt = cnt + 1;
end
xz_boundary = zeros(3, size(xz_x, 1) + size(xz_x, 2) - 1);
xz_boundary_deformed = xz_boundary;
cnt = 1;
for col = 1 : size(xz_x, 2)
    xz_boundary(:, cnt) = [xz_x(1, col); xz_y(1, col); xz_z(1, col)];
    xz_boundary_deformed(:, cnt) = [xz_x(1, col); xz_y(1, col); xz_z_deformed(1, col)];
    cnt = cnt + 1;
end
for row = 2 : size(xz_x, 1)
    xz_boundary(:, cnt) = [xz_x(row, end); xz_y(row, end); xz_z(row, end)];
    xz_boundary_deformed(:, cnt) = [xz_x(row, end); xz_y(row, end); xz_z_deformed(row, end)];
    cnt = cnt + 1;
end
boundary_set = [yz_boundary, xy_boundary, xz_boundary];
boundary_set_deformed = [yz_boundary_deformed, xy_boundary_deformed, xz_boundary_deformed];

%% Find pts with maximum displacements
boundary_pts_num = size(boundary_set, 2);
boundary_displacement = zeros(1, boundary_pts_num);
for i = 1 : boundary_pts_num
    boundary_displacement(1, i) = norm(boundary_set_deformed(:, i) - boundary_set(:, i));
end

[dis_max, dis_max_idx] = max(boundary_displacement);
threshold = 0.05;
dis_max_idx_set = [];
for i = 1 : boundary_pts_num
    if abs(boundary_displacement(1, i) - boundary_displacement(1, dis_max_idx)) <= threshold
        dis_max_idx_set = [dis_max_idx_set, i];
    end
end
s_g_avg = sum(boundary_set(:, dis_max_idx_set), 2) / size(dis_max_idx_set, 2);
s_g_dis_to_boundary = zeros(1, boundary_pts_num);
for i = 1 : boundary_pts_num
    s_g_dis_to_boundary(1, i) = norm(s_g_avg - boundary_set(:, i));
end
[~, min_dis_idx] = min(s_g_dis_to_boundary);
s_g_proj = boundary_set_deformed(:, min_dis_idx);


%% Displacement-weighted s_g
dis_sum = sum(boundary_displacement, 2);
dis_sum_quad = boundary_displacement * boundary_displacement';
s_g_dis_weighted = zeros(3, 1);
s_g_dis_weighted_quad = zeros(3, 1);
for i = 1 : boundary_pts_num
    dis_weight = boundary_displacement(1, i) / dis_sum;
    dis_weight_quad = boundary_displacement(1, i)^2 / dis_sum_quad;
    s_g_dis_weighted = s_g_dis_weighted + boundary_set(:, i) * dis_weight;
    s_g_dis_weighted_quad = s_g_dis_weighted_quad + boundary_set(:, i) * dis_weight_quad;
end
s_g_dis_weighted_to_boundary = zeros(1, boundary_pts_num);
s_g_dis_weighted_quad_to_boundary = zeros(1, boundary_pts_num);
for i = 1 : boundary_pts_num
    s_g_dis_weighted_to_boundary(1, i) = norm(s_g_dis_weighted - boundary_set(:, i));
    s_g_dis_weighted_quad_to_boundary(1, i) = norm(s_g_dis_weighted_quad - boundary_set(:, i));
end
[~, min_dis_idx] = min(s_g_dis_weighted_to_boundary);
s_g_dis_weighted_proj = boundary_set_deformed(:, min_dis_idx);
[~, min_dis_idx] = min(s_g_dis_weighted_quad_to_boundary);
s_g_dis_weighted_quad_proj = boundary_set_deformed(:, min_dis_idx);


%%
% xy_z_deformed_log = xy_z_deformed;
% xz_z_deformed_log = xz_z_deformed;
% save('flexion_xy_z_0517.mat', 'xy_z_deformed_log');
% save('flexion_xz_z_0517.mat', 'xz_z_deformed_log');
% s_g_proj_log = s_g_proj;
% s_g_dis_weighted_proj_log = s_g_dis_weighted_proj;
% s_g_dis_weighted_quad_proj_log = s_g_dis_weighted_quad_proj;
% save('flexion_s_g_max_dis_0517.mat', 's_g_proj_log');
% save('flexion_s_g_dis_weight_0517.mat', 's_g_dis_weighted_proj_log');
% save('flexion_s_g_dis_weight_quad_0517.mat', 's_g_dis_weighted_quad_proj_log')

%% plot
load('flexion_xy_z_0517.mat')
load('flexion_xz_z_0517.mat')
load('flexion_s_g_max_dis_0517.mat')
load('flexion_s_g_dis_weight_0517.mat')
load('flexion_s_g_dis_weight_quad_0517.mat')
fig1 = figure('Position', [300, 300, 1100, 480]);
h1 = subplot(1, 2, 1);
for row = 1 : size(xy_x, 1)
    scatter3(xy_x(row, :), xy_y(row, :), xy_z(row, :), ...
             'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 1);
    hold on
    scatter3(xy_x(row, :), xy_y(row, :), xy_z_deformed_log(row, :), ...
             'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.8);
    hold on
end
for row = 1 : size(xz_x, 1)
    scatter3(xz_x(row, :), xz_y(row, :), xz_z(row, :), ...
             'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k');
    hold on
    for col = 1 : size(xz_x, 2)
        if xz_z_deformed(row, col) >= 20
            scatter3(xz_x(row, col), xz_y(row, col), xz_z_deformed_log(row, col), ...
                 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.8);    
             hold on
        end
    end
end
for row = 1 : size(yz_x, 1)
    scatter3(yz_x(row, :), yz_y(row, :), yz_z(row, :), ...
             'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k');
    hold on;   
end
hold on
scatter3(s_g_proj_log(1, 1), s_g_proj_log(2, 1), s_g_proj_log(3, 1) + 1, 40, ...
        'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k')
hold on 
quiver3(s_g_proj_log(1, 1), s_g_proj_log(2, 1), s_g_proj_log(3, 1), 0, 0, 20, ...
        'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.6)
text(s_g_proj_log(1, 1) - 15, s_g_proj_log(2, 1) + 2, s_g_proj_log(3, 1) + 5, 's_{g,d}', 'FontSize', 17, 'Color', 'w')    
hold on

scatter3(s_g_dis_weighted_proj_log(1, 1), s_g_dis_weighted_proj_log(2, 1), s_g_dis_weighted_proj_log(3, 1) + 1, 40, ...
        'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k')
hold on
quiver3(s_g_dis_weighted_proj_log(1, 1), s_g_dis_weighted_proj_log(2, 1), s_g_dis_weighted_proj_log(3, 1), 0, 0, 20, ...
        'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.6)
text(s_g_dis_weighted_proj_log(1, 1) - 12, s_g_dis_weighted_proj_log(2, 1), s_g_dis_weighted_proj_log(3, 1) + 2, 's_g^1',...
    'FontSize', 17, 'Color', 'w')
hold on

% scatter3(s_g_dis_weighted_quad_proj_log(1, 1), s_g_dis_weighted_quad_proj_log(2, 1), s_g_dis_weighted_quad_proj_log(3, 1) + 1, 40, ...
%         'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k')
% hold on
% quiver3(s_g_dis_weighted_quad_proj_log(1, 1), s_g_dis_weighted_quad_proj_log(2, 1), s_g_dis_weighted_quad_proj_log(3, 1), 0, 0, 20, ...
%         'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.6)
% text(s_g_dis_weighted_quad_proj_log(1, 1) - 9, s_g_dis_weighted_quad_proj_log(2, 1), s_g_dis_weighted_quad_proj_log(3, 1), 's_g^2',...
%     'FontSize', 14, 'Color', 'w')
axis equal
axis([0, 100, 0, 50, 0, 80])
ax = gca;
ax.FontSize = 14;
xlabel('x', 'FontSize', 18)
ylabel('y', 'FontSize', 18)
zlabel('z', 'FontSize', 18)
set(h1, 'Position', [0.08, 0.15, 0.4, 0.8])

h2 = subplot(1, 2, 2);
for row = 1 : size(xy_x, 1)
    scatter3(xy_x(row, :), xy_y(row, :), xy_z(row, :), ...
             'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 1);
    hold on;
    scatter3(xy_x(row, :), xy_y(row, :), xy_z_deformed(row, :), ...
             'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.8);
    hold on;    
end
for row = 1 : size(xz_x, 1)
    scatter3(xz_x(row, :), xz_y(row, :), xz_z(row, :), ...
             'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k');
    hold on;
    for col  = 1 : size(xz_x, 2)
        if xz_z_deformed(row, col) >= 20
            scatter3(xz_x(row, col), xz_y(row, col), xz_z_deformed(row, col), ...
                 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.8);    
        end
    end
end
for row = 1 : size(yz_x, 1)
    scatter3(yz_x(row, :), yz_y(row, :), yz_z(row, :), ...
             'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k');
    hold on;  
    for col = 1 : size(yz_x, 2)
        if yz_z_deformed(row, col) >= 20
            scatter3(yz_x(row, col), yz_y(row, col), yz_z_deformed(row, col), ...
             'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'k',  'MarkerFaceAlpha', 0.8);
        end
    end    
end
hold on
scatter3(s_g_proj(1, 1), s_g_proj(2, 1), s_g_proj(3, 1) + 1, 40, ...
        'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k')
hold on 
quiver3(s_g_proj(1, 1), s_g_proj(2, 1), s_g_proj(3, 1), 0, 0, 12, ...
        'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.6)
text(s_g_proj(1, 1) - 12, s_g_proj(2, 1) - 7, s_g_proj(3, 1), 's_{g,d}', 'FontSize', 17, 'Color', 'w')   
hold on

scatter3(s_g_dis_weighted_proj(1, 1), s_g_dis_weighted_proj(2, 1), s_g_dis_weighted_proj(3, 1) + 1, 40, ...
        'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k')
hold on
quiver3(s_g_dis_weighted_proj(1, 1), s_g_dis_weighted_proj(2, 1), s_g_dis_weighted_proj(3, 1), 0, 0, 20, ...
        'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.6)
text(s_g_dis_weighted_proj(1, 1) - 12, s_g_dis_weighted_proj(2, 1), s_g_dis_weighted_proj(3, 1) + 2, 's_g^1',...
    'FontSize', 17, 'Color', 'w')
hold on

scatter3(s_g_dis_weighted_quad_proj(1, 1), s_g_dis_weighted_quad_proj(2, 1), s_g_dis_weighted_quad_proj(3, 1) + 1, 40, ...
        'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k')
hold on
quiver3(s_g_dis_weighted_quad_proj(1, 1), s_g_dis_weighted_quad_proj(2, 1), s_g_dis_weighted_quad_proj(3, 1), 0, 0, 20, ...
        'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.6)
text(s_g_dis_weighted_quad_proj(1, 1) - 12, s_g_dis_weighted_quad_proj(2, 1), s_g_dis_weighted_quad_proj(3, 1) + 2, 's_g^2',...
    'FontSize', 17, 'Color', 'w')
hold on

axis equal
axis([0, 100, 0, 50, 0, 80])
ax = gca;
ax.FontSize = 14;
xlabel('x', 'FontSize', 18)
ylabel('y', 'FontSize', 18)
zlabel('z', 'FontSize', 18)
set(h2, 'Position', [0.56, 0.15, 0.4, 0.8])
set(gcf, 'Renderer', 'Painters');
print(fig1, './Figure/point_cloud_sim_single_grasp', '-depsc')