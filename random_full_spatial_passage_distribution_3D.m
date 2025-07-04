clear all
close all
addpath('visibility_lib/')
addpath('visibility_lib/p_poly_dist')

%rng(2, "twister");  % seed 1 or 7 is good.
omap3D =  occupancyMap3D;
map_length = 1000;
map_width = 600;
map_height = 400;
obs_num = 20;

obs_idx = 1;
obs_heights = zeros(1, obs_num);
obs_centers = zeros(2, obs_num);
obs_array = createArray(1, obs_num, 'polyshape');
while obs_idx <= obs_num
    width = randi([20, 60], 1);                                                                 
    length = randi([20, 60], 1);           
    height = randi([20, map_height], 1);
    % left-bottom corner position of the obstacle section
    x_pos = randi([0 map_length - width], 1);
    y_pos = randi([0 map_width - length], 1);
    
    %% 
    origin_vertices = [0, width, width, 0;
                       0, 0, length, length];
    rotate_ang = rand(1) * 2 * pi;
    obs_vertices = [cos(rotate_ang), -sin(rotate_ang);
                    sin(rotate_ang), cos(rotate_ang)] * origin_vertices;
    obs_vertices = [x_pos; y_pos] + obs_vertices;
    out_of_box = false;
    for i = 1 : 4
        if obs_vertices(1, i) < 0 || obs_vertices(1, i) > map_length ...
           || obs_vertices(2, i) < 0 || obs_vertices(2, i) > map_width
        out_of_box = true;
        break
        end
    end
    if out_of_box
        continue
    end
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
obs_heights(1, 1) = 400;

%% 
height_idx_map = obs_heights;
height_idx_map = [height_idx_map; 1 : obs_num];
height_idx_map = height_idx_map';
height_idx_map = sortrows(height_idx_map, 'descend');
height_idx_map = height_idx_map';
height_idx_map = [height_idx_map, [0; 0]];

height_psg_array = createArray(1, obs_num, 'height_passages');
for height_idx = 2 : obs_num
    obs_idx = height_idx_map(2, 1 : height_idx);
    test_obs_array = obs_array(obs_idx);
    [valid_psg_pair, ~, psg_pts, ~] = extendedVisibilityCheck(test_obs_array);
    valid_psg_pair = obs_idx(valid_psg_pair);
    height_low = height_idx_map(1, height_idx + 1);
    height_high = height_idx_map(1, height_idx);

    height_psg_array(1, height_idx).height_interval = [height_low, height_high];
    height_psg_array(1, height_idx).passage_pairs = valid_psg_pair;
    height_psg_array(1, height_idx).passage_pts = psg_pts;
end

%% Manage the final result [obs_idx1, obs_idx2, passage_pt1, passage_pt2, valid_height_low, valid_height_high]
psg_num = size(height_psg_array(end).passage_pairs, 1);
psg_info = zeros(psg_num, 8);
psg_info(:, 1:2) = height_psg_array(end).passage_pairs;
psg_info(:, 3:6) = height_psg_array(end).passage_pts;
psg_info(:, 7:8) = repmat(height_psg_array(end).height_interval, psg_num, 1);

for i = obs_num - 1 : -1 : 2
    interval_psg_num = size(height_psg_array(1, i).passage_pairs, 1);
    for j = 1 : interval_psg_num
        psg_pair = height_psg_array(1, i).passage_pairs(j, :);
        psg_pts = height_psg_array(1, i).passage_pts(j, :);
        h = height_psg_array(1, i).height_interval;
        existing = ismember(psg_info(:, 1:2), psg_pair, "rows");
        if nnz(existing)
            [row_idx, ~] = find(existing);
            psg_info(row_idx, end) = h(1, 2);
        else
            new_row = [psg_pair, psg_pts, h];
            psg_info = [psg_info; new_row];
        end           
    end
end

%% plotting 3D map
fig_1 = figure('Position', [701 286 600 400]*2);
cur_axes = show(omap3D);
% ground
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
for i = 1 : size(psg_info, 1)
    hold on
    cur_x = [psg_info(i, 3), psg_info(i, 3), psg_info(i, 5), psg_info(i, 5)];
    cur_y = [psg_info(i, 4), psg_info(i, 4), psg_info(i, 6), psg_info(i, 6)];
    cur_z = [psg_info(i, 7), psg_info(i, 8), psg_info(i, 8), psg_info(i, 7)];
    fill3(cur_x, cur_y, cur_z, 'r', 'FaceAlpha', 0.4, 'EdgeColor', 'r')
end    
% for height_idx = 2 : obs_num
%     passage_pts = height_passage_array(1, height_idx).passage_pts;
%     height_low = height_passage_array(1, height_idx).height_interval(1, 1);
%     height_high = height_passage_array(1, height_idx).height_interval(1, 2);
%     for i = 1 : size(passage_pts, 1)
%         hold on
%         cur_x = [passage_pts(i, 1), passage_pts(i, 1), passage_pts(i, 3), passage_pts(i, 3)];
%         cur_y = [passage_pts(i, 2), passage_pts(i, 2), passage_pts(i, 4), passage_pts(i, 4)];
%         cur_z = [height_low, height_high, height_high, height_low];
%         fill3(cur_x, cur_y, cur_z, 'r', 'FaceAlpha', 0.4, 'EdgeColor', 'r')
%     end  
% end
cur_axes.View = [-41.6283   37.5455];
colormap Gray
cur_axes.CLim = [0, 0.1];
cur_axes.Title.String = '';
cur_axes.XLabel.String = '';
cur_axes.YLabel.String = '';
cur_axes.ZLabel.String = '';
cur_axes.FontSize = 14;
cur_axes.XTick = 0:200:map_length;
cur_axes.YTick = 0:200:map_width;
cur_axes.ZTick = 0:200:map_height;
axis('equal')
% material dull
% set(gcf, 'Renderer', 'Painters')
% print(fig_1, './Figures/3d_map_1214_0', '-depsc')

%%
%% plotting 3D map
fig_2 = figure('Position', [701 286 600 400]*2);
cur_axes = show(omap3D);
% ground
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
for height_idx = obs_num : obs_num
    passage_pts = height_psg_array(1, height_idx).passage_pts;
    height_low = height_psg_array(1, height_idx).height_interval(1, 1);
    height_high = height_psg_array(1, height_idx).height_interval(1, 2);
    for i = 1 : size(passage_pts, 1)
        hold on
        cur_height = min(obs_heights(height_psg_array(1, height_idx).passage_pairs(i, :)));
        cur_x = [passage_pts(i, 1), passage_pts(i, 1), passage_pts(i, 3), passage_pts(i, 3)];
        cur_y = [passage_pts(i, 2), passage_pts(i, 2), passage_pts(i, 4), passage_pts(i, 4)];
        cur_z = [0, cur_height, cur_height, 0];
        fill3(cur_x, cur_y, cur_z, 'r', 'FaceAlpha', 0.4, 'EdgeColor', 'r')
    end  
end
cur_axes.View = [-41.6283   37.5455];
colormap Gray
cur_axes.CLim = [0, 0.1];
cur_axes.Title.String = '';
cur_axes.XLabel.String = '';
cur_axes.YLabel.String = '';
cur_axes.ZLabel.String = '';
cur_axes.FontSize = 14;
cur_axes.XTick = 0:200:map_length;
cur_axes.YTick = 0:200:map_width;
cur_axes.ZTick = 0:200:map_height;
axis('equal')
% material dull
% set(gcf, 'Renderer', 'Painters')
% print(fig_2, './Figures/3d_map_1214_0', '-depsc')

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
            
            % d_min_i_j: distances to obs_j
            % x_min_i_j, y_min_i_j are point coordinates on polygon obs_j closest to obs_i vertices 
            [d_min_i_j, x_min_i_j, y_min_i_j] = p_poly_dist(x_obs_i, y_obs_i, x_obs_j, y_obs_j, true);
            [d_min_j_i, x_min_j_i, y_min_j_i] = p_poly_dist(x_obs_j, y_obs_j, x_obs_i, y_obs_i, true);
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
            
            %% 
            psg_direction = passage_pt_1 - passage_pt_2;
            psg_direction = psg_direction / norm(psg_direction);
            psg_norm = [-psg_direction(1, 2), psg_direction(1, 1)];
            
            max_proj_i = 0;
            max_proj_vertex_i = zeros(1, 2);
            min_proj_i = 0;
            min_proj_vertex_i = zeros(1, 2);
            vertex_i = obs_array(i).Vertices;
            vertex_num = size(vertex_i, 1);
            for vertex_idx = 1 : vertex_num
                vertex = vertex_i(vertex_idx, :);
                vertex_vec = vertex - passage_center;
                inner_product = vertex_vec(1, 1) * psg_norm(1, 1) + vertex_vec(1, 2) * psg_norm(1, 2);
                if inner_product >= max_proj_i
                    max_proj_i = inner_product;
                    max_proj_vertex_i = vertex;
                end
                if inner_product <= min_proj_i
                    min_proj_i = inner_product;
                    min_proj_vertex_i = vertex;
                end
            end

            max_proj_j = 0;
            max_proj_vertex_j = zeros(1, 2);
            min_proj_j = 0;
            min_proj_vertex_j = zeros(1, 2);
            vertex_j = obs_array(j).Vertices;
            vertex_num = size(vertex_j, 1);
            for vertex_idx = 1 : vertex_num
                vertex = vertex_j(vertex_idx, :);
                vertex_vec = vertex - passage_center;
                inner_product = vertex_vec(1, 1) * psg_norm(1, 1) + vertex_vec(1, 2) * psg_norm(1, 2);
                if inner_product >= max_proj_j
                    max_proj_j = inner_product;
                    max_proj_vertex_j = vertex;
                end
                if inner_product <= min_proj_j
                    min_proj_j = inner_product;
                    min_proj_vertex_j = vertex;
                end
            end
            
            % close the vertex
            vertex_i = [vertex_i; vertex_i(1, :)];
            vertex_j = [vertex_j; vertex_j(1, :)];

            max_proj_pt_i = zeros(1, 2);
            max_proj_pt_j = zeros(1, 2);
            if max_proj_i < max_proj_j
                max_proj_pt_i = max_proj_vertex_i;
                
                max_proj_line_end_1 = max_proj_pt_i + psg_direction * 1000;
                max_proj_line_end_2 = max_proj_pt_i - psg_direction * 1000;
                line_x = [max_proj_line_end_1(1, 1), max_proj_line_end_2(1, 1)];
                line_y = [max_proj_line_end_1(1, 2), max_proj_line_end_2(1, 2)];
                [xi, yi] = polyxpoly(line_x, line_y, vertex_j(:, 1)', vertex_j(:, 2)');
                min_dist_to_i = 1e6;
                for idx = 1 : size(xi, 1)
                    intersect_pt = [xi(idx, 1), yi(idx, 1)];
                    dist = norm(intersect_pt - max_proj_pt_i);
                    if dist < min_dist_to_i
                        max_proj_pt_j = intersect_pt;
                        min_dist_to_i = dist;
                    end
                end
            else
                max_proj_pt_j = max_proj_vertex_j;
                
                max_proj_line_end_1 = max_proj_pt_j + psg_direction * 1000;
                max_proj_line_end_2 = max_proj_pt_j - psg_direction * 1000;
                line_x = [max_proj_line_end_1(1, 1), max_proj_line_end_2(1, 1)];
                line_y = [max_proj_line_end_1(1, 2), max_proj_line_end_2(1, 2)];
                [xi, yi] = polyxpoly(line_x, line_y, vertex_i(:, 1)', vertex_i(:, 2)');
                min_dist_to_j = 1e6;
                for idx = 1 : size(xi, 1)
                    intersect_pt = [xi(idx, 1), yi(idx, 1)];
                    dist = norm(intersect_pt - max_proj_pt_j);
                    if dist < min_dist_to_j
                        max_proj_pt_i = intersect_pt;
                        min_dist_to_j = dist;
                    end
                end
            end

            min_proj_pt_i = zeros(1, 2);
            min_proj_pt_j = zeros(1, 2);
            if min_proj_i > min_proj_j
                min_proj_pt_i = min_proj_vertex_i;
                
                min_proj_line_end_1 = min_proj_pt_i + psg_direction * 1000;
                min_proj_line_end_2 = min_proj_pt_i - psg_direction * 1000;
                line_x = [min_proj_line_end_1(1, 1), min_proj_line_end_2(1, 1)];
                line_y = [min_proj_line_end_1(1, 2), min_proj_line_end_2(1, 2)];
                [xi, yi] = polyxpoly(line_x, line_y, vertex_j(:, 1)', vertex_j(:, 2)');
                min_dist_to_i = 1e6;
                for idx = 1 : size(xi, 1)
                    intersect_pt = [xi(idx, 1), yi(idx, 1)];
                    dist = norm(intersect_pt - min_proj_pt_i);
                    if dist < min_dist_to_i
                        min_proj_pt_j = intersect_pt;
                        min_dist_to_i = dist;
                    end
                end
            else
                min_proj_pt_j = min_proj_vertex_j;
                
                min_proj_line_end_1 = min_proj_pt_j + psg_direction * 1000;
                min_proj_line_end_2 = min_proj_pt_j - psg_direction * 1000;
                line_x = [min_proj_line_end_1(1, 1), min_proj_line_end_2(1, 1)];
                line_y = [min_proj_line_end_1(1, 2), min_proj_line_end_2(1, 2)];
                [xi, yi] = polyxpoly(line_x, line_y, vertex_i(:, 1)', vertex_i(:, 2)');
                min_dist_to_j = 1e6;
                for idx = 1 : size(xi, 1)
                    intersect_pt = [xi(idx, 1), yi(idx, 1)];
                    dist = norm(intersect_pt - min_proj_pt_j);
                    if dist < min_dist_to_j
                        min_proj_pt_i = intersect_pt;
                        min_dist_to_j = dist;
                    end
                end
            end
            
            max_proj_pts = [max_proj_pt_i; max_proj_pt_j];
            min_proj_pts = [min_proj_pt_i; min_proj_pt_j];
            
            %%
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
                                                     x_obs_k, y_obs_k, true);
                    obs_k_dist_to_passage_center = min(dist_to_passage_center);
                    if obs_k_dist_to_passage_center < min_dist / 2
                        is_valid_passage = false;
                        break;
                    end
                end
                
                vertex_k = obs_array(k).Vertices;
                vertex_k = [vertex_k; vertex_k(1, :)];
                %% Passage region intersection check in the Gabriel condition
                [xi, yi] = polyxpoly(max_proj_pts(:, 1)', max_proj_pts(:, 2)', vertex_k(:, 1)', vertex_k(:, 2)');
                if size(xi, 1) > 0
                    is_valid_passage = false;
                    break;
                end

                [xi, yi] = polyxpoly(min_proj_pts(:, 1)', min_proj_pts(:, 2)', vertex_k(:, 1)', vertex_k(:, 2)');
                if size(xi, 1) > 0
                    is_valid_passage = false;
                    break;
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