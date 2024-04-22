clear all
close all
load('./occupancymap_data.mat')

fig_2D = figure('Position', [450 350 1000 450]);
subplot(1, 2, 1)
for i = 1 : valid_passage_visibility_num
    plot([passage_visibility_pts(i, 1), passage_visibility_pts(i, 3)], ...
         [passage_visibility_pts(i, 2), passage_visibility_pts(i, 4)], '--', 'Color', [0 0.4470 0.7410], ...
         'LineWidth', 1)
    hold on
end
for i = 1 : valid_passage_num
    plot([passage_pts(i, 1), passage_pts(i, 3)], ...
         [passage_pts(i, 2), passage_pts(i, 4)], 'Color', 'k', 'LineWidth', 1.5)
    hold on
end
for i = 1 : numberOfObstacles
    plot(obs_array(i), 'FaceColor', [0.66 0.66 0.66], 'FaceAlpha', 1)
    text(obs_center(1, i) - 3, obs_center(2, i) + 1, string(i), 'FontSize', 16)
    hold on
end
axis('equal')
axis([0, 200, 0, 200])
ax = gca;
% ax.FontName = 'Arial';
ax.FontSize = 18;
ax.Box = 'on';
ax.LineWidth = 1.5;
xlabel('x (m)', 'FontSize', 21)
ylabel('y (m)', 'FontSize', 21)
text(135, 180, 'z < 21 m', 'FontSize', 16)

subplot(1, 2, 2)
filtered_obs_num = size(filtered_obs_idx, 2);
for i = 1 : valid_passage_visibility_num
    plot([passage_visibility_pts(i, 1), passage_visibility_pts(i, 3)], ...
         [passage_visibility_pts(i, 2), passage_visibility_pts(i, 4)], '--', 'Color', [0 0.4470 0.7410], ...
         'LineWidth', 1)
    hold on
end
for i = 1 : valid_passage_num
    plot([passage_pts(i, 1), passage_pts(i, 3)], ...
         [passage_pts(i, 2), passage_pts(i, 4)], 'Color', 'k', 'LineWidth', 1.5)
    hold on
end
for i = 1 : filtered_obs_num
    cur_obs_idx = filtered_obs_idx(1, i);
    plot(obs_array(cur_obs_idx), 'FaceColor', [0.66 0.66 0.66], 'FaceAlpha', 1)
    text(obs_center(1, cur_obs_idx) - 3, obs_center(2, cur_obs_idx) + 1, string(cur_obs_idx), 'FontSize', 16)
    hold on
end
axis('equal')
axis([0, 200, 0, 200])
ax = gca;
% ax.FontName = 'Arial';
ax.FontSize = 18;
ax.Box = 'on';
ax.LineWidth = 1.5;
xlabel('x (m)', 'FontSize', 21)
ylabel('y (m)', 'FontSize', 21)
text(120, 180, '55 < z < 65 m', 'FontSize', 16)
set(gcf, 'Renderer', 'Painters')
% print(fig_2D, './Figures/Height_Varying_Passage_Result_0115', '-depsc')

%% plotting 3D map
fig_1 = figure('Position', [701 286 540 420]);
for i = 1 : size(passage_pts, 1)
    hold on
    cur_height = min(obs_height(valid_passage_pair(i, :)));
    cur_x = [passage_pts(i, 1), passage_pts(i, 1), passage_pts(i, 3), passage_pts(i, 3)];
    cur_y = [passage_pts(i, 2), passage_pts(i, 2), passage_pts(i, 4), passage_pts(i, 4)];
    cur_z = [0, cur_height, cur_height, 0];
    fill3(cur_x, cur_y, cur_z, 'r', 'FaceAlpha', 0.4, 'EdgeColor', 'r')
end    
hold on
cur_axes = show(omap3D);
cur_axes.View = [-43.3487, 43.0218];
colormap Gray
cur_axes.CLim = [0, 0.1];
cur_axes.Title.String = '';
cur_axes.XLabel.String = 'x (m)';
cur_axes.YLabel.String = 'y (m)';
cur_axes.ZLabel.String = 'z (m)';
axis('equal')
% set(gcf, 'Renderer', 'Painters')
% print(fig_1, './Figures/3d_map_passage', '-depsc')
% print(fig_1, './Figures/3d_map_passage_0', '-dpng')