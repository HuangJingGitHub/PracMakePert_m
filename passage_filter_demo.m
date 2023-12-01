clear all
close all
clc

addpath('./p_poly_dist')

obs_dimension = [1, 1];  % obs length, width
obs_num = 40;
obs_center = zeros(obs_num, 2);

x_map_limit = [0, 50];
y_map_limit = [0, 25];

x_length = x_map_limit(1, 2) - x_map_limit(1, 1);
y_length = y_map_limit(1, 2) - y_map_limit(1, 1);
for i = 1 : obs_num
   cur_x = rand();
   cur_y = rand();
   cur_x = cur_x * x_length;
   cur_y = cur_y * y_length;
   cur_center = [cur_x, cur_y];
   
   nearest_idx = knnsearch(obs_center, cur_center);
   dist_vec = cur_center - obs_center(nearest_idx, :);
   while dist_vec * dist_vec.' < obs_dimension * obs_dimension.'
       cur_x = rand();
       cur_y = rand();
       cur_x = cur_x * x_length;
       cur_y = cur_y * y_length;
       cur_center = [cur_x, cur_y];

       nearest_idx = knnsearch(obs_center, cur_center);
       dist_vec = cur_center - obs_center(nearest_idx, :);   
   end
   obs_center(i, :) = cur_center;
   
   x_clearance = obs_dimension(1, 1) / 2;
   if cur_x < x_map_limit(1, 1) + x_clearance
        cur_x = x_map_limit(1, 1) + x_clearance;
   elseif cur_x > x_map_limit(1, 2) - x_clearance
       cur_x = x_map_limit(1, 2) - x_clearance;
   end
   
   y_clearance = obs_dimension(1, 2) / 2;
   if cur_y < y_map_limit(1, 1) + y_clearance
       cur_y = y_map_limit(1, 1) + y_clearance;
   elseif cur_y > y_map_limit(1, 2) - y_clearance
       cur_y = y_map_limit(1, 2) - y_clearance;
   end
   
   obs_array(i) = polyshape([cur_x - x_clearance, cur_x + x_clearance, cur_x + x_clearance, cur_x - x_clearance],...
                           [cur_y - y_clearance, cur_y - y_clearance, cur_y + y_clearance, cur_y + y_clearance]);
end
[valid_passage_pair, valid_passage_visibility_pair] = extendedVisibilityCheck(obs_array);
valid_passage_num = size(valid_passage_pair, 1)
valid_passage_visibility_num = size(valid_passage_visibility_pair, 1)

fig = figure('Position', [441 336 1080 540]);
for i = 1 : obs_num
    plot(obs_array(i), 'FaceColor', 'k')
    hold on
end
for i = 1 : valid_passage_visibility_num
    obsIdx1 = valid_passage_visibility_pair(i, 1);
    obsIdx2 = valid_passage_visibility_pair(i, 2);
    plot([obs_center(obsIdx1, 1), obs_center(obsIdx2, 1)], ...
         [obs_center(obsIdx1, 2), obs_center(obsIdx2, 2)], '--k')
end
for i = 1 : valid_passage_num
    obsIdx1 = valid_passage_pair(i, 1);
    obsIdx2 = valid_passage_pair(i, 2);
    plot([obs_center(obsIdx1, 1), obs_center(obsIdx2, 1)], ...
         [obs_center(obsIdx1, 2), obs_center(obsIdx2, 2)], '--r')
end
axis('equal')
axis([x_map_limit(1, 1), x_map_limit(1, 2), y_map_limit(1, 1), y_map_limit(1, 2)])


%%
function [valid_passage_pair, valid_passage_visibility] = extendedVisibilityCheck(obs_array)
    valid_passage_pair = [];
    valid_passage_visibility = [];
    
    obs_num = size(obs_array, 2);
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
                x_d_min_pair = [x_obs_j(1, d_min_idx_j_i), x_min_j_i(d_min_idx_i_j, 1)];
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
            end
            if is_valid_passage_visibility == true
                valid_passage_visibility = [valid_passage_visibility; i, j];
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
