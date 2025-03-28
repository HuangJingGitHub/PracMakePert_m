clear all
close all
addpath('visibility_lib/')
addpath('visibility_lib/p_poly_dist')

%rng(2, "twister");  % seed 1 or 7 is good.
omap3D =  occupancyMap3D;
map_length = 1000;
map_width = 600;
map_height = 400;
obs_num = 50;

obs_idx = 1;
obs_heights = zeros(1, obs_num);
obs_centers = zeros(2, obs_num);
obs_array = createArray(1, obs_num, 'polyshape');
while obs_idx <= obs_num
    width = randi([10, 60], 1);                                                                 
    length = randi([10, 60], 1);           
    height = randi([30, map_height], 1);
    % left-bottom corner position of the obstacle section
    x_pos = randi([0 map_length - width], 1);
    y_pos = randi([0 map_width - length], 1);
    
    %% 
    obs_vertices = [x_pos, x_pos + width, x_pos + width, x_pos;
                    y_pos, y_pos, y_pos + length, y_pos + length];
    obs_poly = polyshape(obs_vertices(1, :), obs_vertices(2, :));
    in_intersection = false;
    for i = 1 : obs_idx - 1
        intersect_poly = intersect(obs_poly, obs_array(1, i));
        if intersect_poly.NumRegions > 0
            in_intersection = true;
            break;
        end
    end
    if in_intersection
        continue
    end

    obs_array(1, obs_idx) = obs_poly;
    obs_heights(1, obs_idx) = height;
    obs_centers(:, obs_idx) = [x_pos + width / 2; y_pos + length / 2];
    obs_idx = obs_idx + 1; 
end

%% This part is for 2D height-layer passage analysis and plotting
% [valid_passage_pair, valid_passage_visibility_pair, passage_pts, passage_visibility_pts] = extendedVisibilityCheck(obs_array);
% valid_passage_num = size(valid_passage_pair, 1);
% valid_passage_visibility_num = size(valid_passage_visibility_pair, 1);

%% plotting 3D map
fig_1 = figure('Position', [701 286 600 400]*2);
cur_axes = show(omap3D);
% cur_axes = gca;
p = patch('XData', [0, map_length, map_length, 0, 0], 'YData', [0, 0, map_width, map_width, 0], ...
          'ZData', [0, 0, 0, 0, 0]);
p.FaceColor = [0.827, 0.827, 0.827]; 

idx = [1, 2, 3, 4, 1; 1, 2, 6, 5, 1; 2, 3, 7, 6, 2; 3, 4, 8, 7, 3; 1, 4, 8, 5, 1; 5, 6, 7, 8, 5]';
for i = 1 : obs_num
    hold on
    ground_pos = obs_array(1, i).Vertices;
    h = obs_heights(1, i);
    cuboid = [ground_pos(1, :), 0;
             ground_pos(2, :), 0;
             ground_pos(3, :), 0;
             ground_pos(4, :), 0;
             ground_pos(1, :), h;
             ground_pos(2, :), h;
             ground_pos(3, :), h;
             ground_pos(4, :), h;             
             ];
   xc = cuboid(:, 1);
   yc = cuboid(:, 2);
   zc = cuboid(:, 3);
   p = patch('XData', xc(idx), 'YData', yc(idx), 'ZData', zc(idx));
   p.FaceColor = [0.827, 0.827, 0.827]; % light gray
   % p.FaceColor = [0.91, 0.91, 0.91]; 
   % p.LineStyle = 'none';
end
cur_axes.View = [-41.6283   37.5455];
colormap Gray
cur_axes.CLim = [0, 0.1];
cur_axes.Title.String = '';
cur_axes.XLabel.String = '';
cur_axes.YLabel.String = '';
cur_axes.ZLabel.String = '';
cur_axes.XTick = 0:200:map_length;
cur_axes.YTick = 0:200:map_width;
cur_axes.ZTick = 0:200:map_height + 20;
axis('equal')
% material dull
% set(gcf, 'Renderer', 'Painters')
% print(fig_1, './Figures/3d_map_1214_0', '-depsc')


%% Required functions
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