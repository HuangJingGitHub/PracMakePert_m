clear all
close all
clc

addpath(genpath('./visibility_lib/'))

obs_dimension = [5, 5];  % obstacle length, width
obs_num = 15;
obs_type_num = 3;
obs_center = zeros(obs_num, 2);
obs_type = randi([1, obs_type_num], obs_num, 1);
std_obs_vertices{1, 1} = [-0.5, 0.5, 0.5, -0.5;
                          -0.5, -0.5, 0.5, 0.5] * obs_dimension(1, 1);
std_obs_vertices{1, 2} = [-0.25, 0.25, 0.25, -0.25;
                          -0.5, -0.5, 0.5, 0.5] * obs_dimension(1, 1);   
std_obs_vertices{1, 3} = [-0.5, 0.5, 0;
                          -sqrt(3) / 6, -sqrt(3) / 6, sqrt(3) / 3] * obs_dimension(1, 1);
x_map_limit = [0, 50];
y_map_limit = [0, 30];

x_length = x_map_limit(1, 2) - x_map_limit(1, 1);
y_length = y_map_limit(1, 2) - y_map_limit(1, 1);
obs_array = repmat(polyshape(), 1, obs_num); % preallocate array of objects using repmat
for i = 1 : obs_num
    cur_x = rand();
    cur_y = rand();
    cur_x = cur_x * x_length;
    cur_y = cur_y * y_length;
    cur_center = [cur_x, cur_y];

    nearest_idx = knnsearch(obs_center, cur_center);
    dist_vec = cur_center - obs_center(nearest_idx, :);
    while i > 1 && (abs(dist_vec(1, 1)) <= 0.85 * obs_dimension(1, 1) || abs(dist_vec(1, 2)) <= 0.85 * obs_dimension(1, 2))
        cur_x = rand();
        cur_y = rand();
        cur_x = cur_x * x_length;
        cur_y = cur_y * y_length;
        cur_center = [cur_x, cur_y];

        nearest_idx = knnsearch(obs_center, cur_center);
        dist_vec = cur_center - obs_center(nearest_idx, :);
   end

   x_clearance = obs_dimension(1, 1) / 2;
    if cur_x < x_map_limit(1, 1) + x_clearance
        cur_x = x_map_limit(1, 1) + x_clearance * 2;
    elseif cur_x > x_map_limit(1, 2) - x_clearance
        cur_x = x_map_limit(1, 2) - x_clearance * 2;
    end
    y_clearance = obs_dimension(1, 2) / 2;
    if cur_y < y_map_limit(1, 1) + y_clearance
        cur_y = y_map_limit(1, 1) + y_clearance * 2;
    elseif cur_y > y_map_limit(1, 2) - y_clearance
        cur_y = y_map_limit(1, 2) - y_clearance * 2;
    end
    obs_center(i, :) = [cur_x, cur_y];

    cur_rotate_angle = rand() * 360;
    cur_rotz = rotz(cur_rotate_angle);
    cur_rotz = cur_rotz(1:2, 1:2);
    rotated_obs_pos = cur_rotz * std_obs_vertices{1, obs_type(i, 1)};
    rotated_obs_pos(1, :) = rotated_obs_pos(1, :) + cur_x;
    rotated_obs_pos(2, :) = rotated_obs_pos(2, :) + cur_y; 
    obs_array(i) = polyshape(rotated_obs_pos(1, :), rotated_obs_pos(2,:));
end
[valid_passage_pair, valid_passage_visibility_pair, passage_pts, passage_visibility_pts] = extendedVisibilityCheck(obs_array);
valid_passage_num = size(valid_passage_pair, 1)
valid_passage_visibility_num = size(valid_passage_visibility_pair, 1)

%%
fig_1 = figure('Position', [450 350 1000 600]);
% for i = 1 : valid_passage_visibility_num
%     plot([passage_visibility_pts(i, 1), passage_visibility_pts(i, 3)], ...
%          [passage_visibility_pts(i, 2), passage_visibility_pts(i, 4)], '--', 'Color', [0 0.4470 0.7410], ...
%          'LineWidth', 1)
%     hold on
% end
for i = 1 : valid_passage_num
    plot([passage_pts(i, 1), passage_pts(i, 3)], ...
         [passage_pts(i, 2), passage_pts(i, 4)], '--k', 'LineWidth', 1.5)
    hold on
end
for i = 1 : obs_num
    if obs_type(i) == 1
        plot(obs_array(i), 'FaceColor', [0.4660 0.6740 0.1880], 'FaceAlpha', 1)
        text(obs_center(i, 1) - 0.5, obs_center(i, 2), string(i), 'FontSize', 17)
    elseif obs_type(i) == 2
        plot(obs_array(i), 'FaceColor', [0.8500 0.3250 0.0980], 'FaceAlpha', 1)
        text(obs_center(i, 1) - 0.5, obs_center(i, 2), string(i), 'FontSize', 17)
    else
        plot(obs_array(i), 'FaceColor', [0.9290 0.6940 0.1250], 'FaceAlpha', 1)
        text(obs_center(i, 1) - 0.7, obs_center(i, 2), string(i), 'FontSize', 17)
    end
    hold on
end
axis('equal')
axis([x_map_limit(1, 1), x_map_limit(1, 2), y_map_limit(1, 1), y_map_limit(1, 2)])
ax = gca;
ax.XTick = [];
ax.YTick = [];
% ax.FontName = 'Arial';
%ax.FontSize = 18;
% ax.Box = 'on';
% ax.LineWidth = 1.5;
% xlabel('x', 'FontSize', 21)
% ylabel('y', 'FontSize', 21)
set(gcf, 'Renderer', 'Painters')
% print(fig_1, './Figures/Obstacle_Filter_Setup_1202', '-depsc')


%%
function [valid_passage_pair, valid_passage_visibility_pair, passage_pair_pts, passage_pair_visibility_pts] = extendedVisibilityCheck(obs_array)
    valid_passage_pair = [];
    valid_passage_visibility_pair = [];
    passage_pair_pts = [];
    passage_pair_visibility_pts = [];
    
    obs_num = length(obs_array);
    for i = 1 : obs_num - 1
        for j = i + 1 : obs_num
            x_obs_i = obs_array(i).Vertices(:, 1).';
            y_obs_i = obs_array(i).Vertices(:, 2).';
            x_obs_j = obs_array(j).Vertices(:, 1).';
            y_obs_j = obs_array(j).Vertices(:, 2).';
            [d_min_i_j, x_min_i_j, y_min_i_j] = p_poly_dist(x_obs_i, y_obs_i, x_obs_j, y_obs_j);
            [d_min_j_i, x_min_j_i, y_min_j_i] = p_poly_dist(x_obs_j, y_obs_j, x_obs_i, y_obs_i);
            [d_min_val_i_j, d_min_idx_i_j] = min(d_min_i_j);
            [d_min_val_j_i, d_min_idx_j_i] = min(d_min_j_i);
            x_d_min_pair = zeros(1, 2);
            y_d_min_pair = zeros(1, 2);
            min_dist = 0;
            if d_min_val_i_j < d_min_val_j_i
                min_dist = d_min_val_i_j;
                x_d_min_pair = [x_obs_i(1, d_min_idx_i_j), x_min_i_j(d_min_idx_i_j, 1)];
                y_d_min_pair = [y_obs_i(1, d_min_idx_i_j), y_min_i_j(d_min_idx_i_j, 1)];
            else
                min_dist = d_min_val_j_i;
                x_d_min_pair = [x_obs_j(1, d_min_idx_j_i), x_min_j_i(d_min_idx_j_i, 1)];
                y_d_min_pair = [y_obs_j(1, d_min_idx_j_i), y_min_j_i(d_min_idx_j_i, 1)];              
            end
            passage_pt_1 = [x_d_min_pair(1, 1), y_d_min_pair(1, 1)];
            passage_pt_2 = [x_d_min_pair(1, 2), y_d_min_pair(1, 2)];
            passage_center = (passage_pt_1 + passage_pt_2) / 2;
            
            is_valid_passage = true;
            is_valid_passage_visibility = true;
            for k = 1 : obs_num
                if i == k || j == k
                    continue
                end
                
                if pureVisibilityCheck(passage_pt_1, passage_pt_2, obs_array(k)) == false
                    is_valid_passage_visibility = false;
                end
                
                if is_valid_passage_visibility == false
                    is_valid_passage = false;
                    break;
                end
                
                if is_valid_passage == true
                    x_obs_k = obs_array(k).Vertices(:, 1).';
                    y_obs_k = obs_array(k).Vertices(:, 2).';
                    [dist_to_passage_center, ~] = p_poly_dist(passage_center(1, 1), passage_center(1, 2),...
                                                     x_obs_k, y_obs_k);
                    obs_k_dist_to_passage_center = min(dist_to_passage_center);
                    if obs_k_dist_to_passage_center < min_dist / 2
                        is_valid_passage = false;
                    end
                end
            end
            
            if is_valid_passage == true
                valid_passage_pair = [valid_passage_pair; i, j];
                passage_pair_pts = [passage_pair_pts; passage_pt_1, passage_pt_2];
            end
            if is_valid_passage_visibility == true
                valid_passage_visibility_pair = [valid_passage_visibility_pair; i, j];
                passage_pair_visibility_pts = [passage_pair_visibility_pts; passage_pt_1, passage_pt_2];
            end
        end
    end
end

%% 
function valid_passage = pureVisibilityCheck(passage_pt_1, passage_pt_2, obs_obj)
    valid_passage = true;
    obs_vertex_num = size(obs_obj.Vertices, 1);
    for i = 1 : obs_vertex_num - 1
        if segmentsIntersectionCheck(passage_pt_1, passage_pt_2, ...
                                     obs_obj.Vertices(i, :), obs_obj.Vertices(i + 1, :)) == true
           valid_passage = false;
        end
    end
    if segmentsIntersectionCheck(passage_pt_1, passage_pt_2, ...
                                obs_obj.Vertices(end, :), obs_obj.Vertices(1, :)) == true
       valid_passage = false;
    end
end
