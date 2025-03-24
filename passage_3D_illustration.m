clear all
close all
addpath('visibility_lib/')
addpath('visibility_lib/p_poly_dist')

% rng(1, "twister");  % seed 1 or 7 is good.
omap3D =  occupancyMap3D;
mapWidth = 200;
mapLength = 200;
numberOfObstacles = 3;

ObstacleIdx = 1;
obs_height = zeros(1, numberOfObstacles);
obs_center = zeros(2, numberOfObstacles);
while ObstacleIdx <= numberOfObstacles
    width = randi(50, 1);                 % The largest integer in the sample intervals for obtaining width, length and height                                                     
    length = randi(50, 1);                % can be changed as necessary to create different occupancy maps.
    height = randi(150, 1);
    xPosition = randi([0 mapWidth - width], 1);
    yPosition = randi([0 mapLength - length], 1);

    width = 30;
    length = 30;
    x_pos = [35, 135, 85];
    y_pos = [115, 115, 55];
    h = [100, 100, 60];
    xPosition = x_pos(1, ObstacleIdx);
    yPosition = y_pos(1, ObstacleIdx);
    height = h(1, ObstacleIdx);
    
    [xObstacle, yObstacle, zObstacle] = meshgrid(xPosition : xPosition + width, yPosition : yPosition + length, 0 : height);
    xyzObstacles = [xObstacle(:) yObstacle(:) zObstacle(:)];
    
    checkIntersection = false;
    for i = 1 : size(xyzObstacles, 1)
        if checkOccupancy(omap3D, xyzObstacles(i,:)) == 1
            checkIntersection = true;
            break
        end
    end
    if checkIntersection
        continue
    end
    
    setOccupancy(omap3D, xyzObstacles, 1)
    ObstacleIdx = ObstacleIdx + 1;
    
    %% 
    obs_vertices = [xPosition, xPosition + width, xPosition + width, xPosition;
                    yPosition, yPosition, yPosition + length, yPosition + length];
    obs_array(ObstacleIdx - 1) = polyshape(obs_vertices(1, :), obs_vertices(2, :));
    obs_height(1, ObstacleIdx - 1) = height;
    obs_center(:, ObstacleIdx - 1) = [xPosition + width / 2; yPosition + length / 2];
end

[xGround, yGround, zGround] = meshgrid(0 : mapWidth, 0 : mapLength, 0);
xyzGround = [xGround(:) yGround(:) zGround(:)];
setOccupancy(omap3D,xyzGround,1)

%% This part is for 2D height-layer passage analysis and plotting
[valid_passage_pair, valid_passage_visibility_pair, passage_pts, passage_visibility_pts] = extendedVisibilityCheck(obs_array);
valid_passage_num = size(valid_passage_pair, 1)
valid_passage_visibility_num = size(valid_passage_visibility_pair, 1)


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
cur_height = 100;
cur_x = [65, 65, 135, 135];
cur_y = [115, 115, 115, 115];
cur_z = [60, 100, 100, 60];
fill3(cur_x, cur_y, cur_z, 'r', 'FaceAlpha', 0.4, 'EdgeColor', 'r')
hold on
cur_axes = show(omap3D);
cur_axes.View = [-43.6453   41.4350]; %[-43.3487, 43.0218];
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