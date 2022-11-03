close all
clear
clc

%% load raw file
% wf:white foam
ptcloud_wf_initial = pcread('./Point Cloud/202206011829_without_normal.ply');
ptcloud_wf_deformed = pcread('./Point Cloud/202206011829_deformed_without_normal.ply');

%% extract object point cloud
row_indices_deformed = GetPtCloudRowIndices(ptcloud_wf_deformed.Location(:, 1));
DO_row_indices_deformed = DetectDORowByDepth(10000, 32000, row_indices_deformed, -0.4, ptcloud_wf_deformed.Location);
ptcloud_DO_deformed = ExtractObjectFromPtCloud(DO_row_indices_deformed(10:end, :), ptcloud_wf_deformed);

%% get boundary pts
% left boundary idx | right boundary idx | left local idx | right lcoal idx
% | undeformed average depth
boundary_info = zeros(size(DO_row_indices_deformed, 1), 5);
boundary_info(:, 1 : 2) = DO_row_indices_deformed;
DO_row_num = size(DO_row_indices_deformed, 1);
xyz_location_deformed = double(ptcloud_wf_deformed.Location);
for row = 1 : size(DO_row_indices_deformed, 1)
    start_row = 1;
    end_row = 1;
    for i = 1 : size(row_indices_deformed, 1)
        if row_indices_deformed(i, 1) <= DO_row_indices_deformed(1, 1)...
           && DO_row_indices_deformed(1, 1) <= row_indices_deformed(i, 2)
            start_row = i;
            break
        end
    end
    for i = start_row + 1 : size(row_indices_deformed, 1)
        if row_indices_deformed(i, 1) <= DO_row_indices_deformed(end, 1)...
           && DO_row_indices_deformed(end, 1) <= row_indices_deformed(i, 2)
            end_row = i;
            break;
        end
    end
    
    depth_avg = sum(xyz_location_deformed(row_indices_deformed(start_row + row - 1, 1) : row_indices_deformed(start_row + row - 1, 1) + 4, 3)) / 5;
    boundary_info(row, end) = depth_avg;
    
%     for i = DO_row_indices_deformed(row, 1) + 1 : DO_row_indices_deformed(row, 1) + 19
%         if (xyz_location(i, 3) > xyz_location(i - 1, 3) && xyz_location(i, 3) > xyz_location(i + 1, 3)) ...
%            || (xyz_location(i, 3) < xyz_location(i - 1, 3) && xyz_location(i, 3) < xyz_location(i + 1, 3))
%             boundary_info(row, 3) = i;
%             break;
%         end
%     end
%     for i = DO_row_indices_deformed(row, 2) - 1 : DO_row_indices_deformed(row, 1) - 19
%         if (xyz_location(i, 3) > xyz_location(i - 1, 3) && xyz_location(i, 3) > xyz_location(i + 1, 3)) ...
%            || (xyz_location(i, 3) < xyz_location(i - 1, 3) && xyz_location(i, 3) < xyz_location(i + 1, 3))
%             boundary_info(row, 4) = i;
%             break;
%         end
%     end
end
boundary_idx_displacement = zeros(DO_row_num * 2, 1);
for i = 1 : DO_row_num
    cur_pt_idx = boundary_info(i, 1);
    boundary_idx_displacement(2 * i - 1, 1) = cur_pt_idx;
    boundary_idx_displacement(2 * i - 1, 2) = abs(xyz_location_deformed(cur_pt_idx, 3) - boundary_info(i, end));
    cur_pt_idx = boundary_info(i, 2);
    boundary_idx_displacement(2 * i, 1) = cur_pt_idx;
    boundary_idx_displacement(2 * i, 2) = abs(xyz_location_deformed(cur_pt_idx, 3) - boundary_info(i, end));    
end
[~, order] = sort(boundary_idx_displacement(:, 2));
boundary_idx_displacement_sorted = boundary_idx_displacement(order, :);
S_g_idx = ChooseDualGraspPositions(boundary_idx_displacement_sorted, xyz_location_deformed);


s_g_1_idx = S_g_idx(1, 1);
s_g_2_idx = S_g_idx(1, 2);
display_color = ptcloud_wf_deformed.Color;
for i = 1 : size(DO_row_indices_deformed, 1)
    for j = DO_row_indices_deformed(i, 1) : DO_row_indices_deformed(i, 2)
        display_color(j, :) = [0, 0, 255];
    end
end
s_g_1_row = 0;
for i = 1 : size(row_indices_deformed, 1)
    if row_indices_deformed(i, 1) <= s_g_1_idx && s_g_1_idx <= row_indices_deformed(i, 2)
        s_g_1_row = i;
        break
    end
end
s_g_1_pos = double(ptcloud_wf_deformed.Location(s_g_1_idx, :));
for i = s_g_1_row - 4 : s_g_1_row + 4
    pt_idx_start = row_indices_deformed(i, 1);
    pt_idx_end = row_indices_deformed(i, 2);
    for pt_idx = pt_idx_start : pt_idx_end
        if norm(s_g_1_pos - double(ptcloud_wf_deformed.Location(pt_idx, :))) < 0.005
            display_color(pt_idx, :) = [255, 0, 0];
        end
    end
end
s_g_2_row = 0;
for i = 1 : size(row_indices_deformed, 1)
    if row_indices_deformed(i, 1) <= s_g_2_idx && s_g_2_idx <= row_indices_deformed(i, 2)
        s_g_2_row = i;
        break
    end
end
s_g_2_pos = double(ptcloud_wf_deformed.Location(s_g_2_idx, :));
for i = s_g_2_row - 4 : s_g_2_row + 4
    pt_idx_start = row_indices_deformed(i, 1);
    pt_idx_end = row_indices_deformed(i, 2);
    for pt_idx = pt_idx_start : pt_idx_end
        if norm(s_g_2_pos - double(ptcloud_wf_deformed.Location(pt_idx, :))) < 0.01
            display_color(pt_idx, :) = [255, 0, 0];
        end
    end
end
display_color(s_g_1_idx, :) = [255, 0, 0];
display_color(s_g_2_idx, :) = [255, 0, 0];
display_ptcloud = pointCloud(ptcloud_wf_deformed.Location, 'Color', display_color);

fig = figure;
pcshow(display_ptcloud)
hold on
pcshow(ptcloud_wf_initial)
%set(gcf, 'Renderer', 'Painters');
%print(fig, './Figure/Ptcloud_foam_0', '-depsc')


function row_start_end_indices = GetPtCloudRowIndices(x_coordinates)
    pt_num = size(x_coordinates, 1);
    row_start_end_indices = zeros(500, 2);
    row_start_end_indices(1, 1) = 1;
    row_idx = 1;
    i = 1;
    while i < pt_num
        if x_coordinates(i, 1) > 0 && x_coordinates(i + 1, 1)  < 0
            row_start_end_indices(row_idx, 2) = i;
            row_start_end_indices(row_idx + 1, 1) = i + 1;
            row_idx = row_idx + 1;
            i = i + 2;
        else
            i = i + 1;
        end
    end
    row_start_end_indices(row_idx, 2) = pt_num;
    row_start_end_indices = row_start_end_indices(1 : row_idx, :);   
end


function ptcloud_extracted = ExtractObjectFromPtCloud(row_indices, ptcloud)
    pt_num = 0;
    for row = 1 : size(row_indices, 1)
        pt_num = pt_num + row_indices(row, 2) - row_indices(row, 1) + 1;
    end
    
    pt_xyz = zeros(pt_num, 3);
    pt_color = zeros(pt_num, 3);
    entire_pt_xyz = double(ptcloud.Location);
    entire_pt_color = double(ptcloud.Color);
    pt_idx = 1;
    for row = 1 : size(row_indices, 1)
        row_pt_num = row_indices(row, 2) - row_indices(row, 1) + 1;
        pt_xyz(pt_idx : pt_idx + row_pt_num - 1, :) = entire_pt_xyz(row_indices(row, 1) : row_indices(row, 2), :);
        pt_color(pt_idx : pt_idx + row_pt_num - 1, :) = entire_pt_color(row_indices(row, 1) : row_indices(row, 2), :);
        pt_idx = pt_idx + row_pt_num;
    end
    
    ptcloud_extracted = pointCloud(pt_xyz, 'Color', pt_color);
end

function DO_row_start_end_indices = DetectDORowByDepth(start_idx_guess, end_idx_guess, row_indices, ...
                                                        depth_threshold, ptcloud_location)
    DO_row_start_end_indices = zeros(50, 2);
    
    start_row = 1;
    end_row = 1;
    for i = 1 : size(row_indices, 1)
        if row_indices(i, 1) <= start_idx_guess && start_idx_guess <= row_indices(i, 2)
            start_row = i;
            break
        end
    end
    for i = start_row + 1 : size(row_indices, 1)
        if row_indices(i, 1) <= end_idx_guess && end_idx_guess <= row_indices(i, 2)
            end_row = i;
            break;
        end
    end
    
    DO_row_idx = 1;
    for row = start_row : end_row
        pt_start_idx = row_indices(row, 1);
        pt_end_idx = row_indices(row, 2);
        
        DO_start_idx = 0;
        for pt_idx = pt_start_idx + 1 : pt_end_idx - 5
            if double(ptcloud_location(pt_idx, 3)) >= depth_threshold ...
               && double(ptcloud_location(pt_idx, 1)) >= -0.15 ...
               && double(ptcloud_location(pt_idx, 1)) <= 0.15
                DO_start_idx = pt_idx;
                break
            end
        end
        DO_end_idx = 0;
        for pt_idx = pt_end_idx - 1 : -1 : pt_start_idx + 5
            if double(ptcloud_location(pt_idx, 3)) >= depth_threshold ...
               && double(ptcloud_location(pt_idx, 1)) >= -0.15 ...
               && double(ptcloud_location(pt_idx, 1)) <= 0.15               
                DO_end_idx = pt_idx;
                break;
            end
        end
        
        mid_idx = floor((DO_start_idx + DO_end_idx) / 2);
        if DO_start_idx ~= 0 && DO_end_idx ~= 0 ...
           && DO_end_idx - DO_start_idx >= 20 ...
           %&& double(ptcloud_location(mid_idx , 3)) >= depth_threshold - 0.01
            DO_row_start_end_indices(DO_row_idx, :) = [DO_start_idx, DO_end_idx];
            DO_row_idx = DO_row_idx + 1;
        end
    end
    DO_row_start_end_indices = DO_row_start_end_indices(1 : DO_row_idx - 1, :);
end


function S_g_idx = ChooseDualGraspPositions(boundary_idx_dis_sorted, xyz_location)
    n_s = 2;
    n = 1;
    s_g_1_idx = boundary_idx_dis_sorted(end, 1);
    s_g_1_pos = xyz_location(s_g_1_idx, :);
    boundary_pt_num = size(boundary_idx_dis_sorted, 1);
    
    B = zeros(boundary_pt_num, 2);
    for i = 1 : boundary_pt_num
        cur_idx = boundary_idx_dis_sorted(i, 1);
        B(i, 1) = cur_idx;
        B(i, 2) = norm(xyz_location(cur_idx, 1 : 2) - s_g_1_pos(1, 1 : 2));
    end
    [~, order] = sort(B(:, 2));
    B = B(order, :);
    
    J_max = 0;
    s_g_2_idx = 0;
    for i = floor(boundary_pt_num / 2) : boundary_pt_num
        cur_idx = B(i, 1);
        cur_dis_to_s_g_1 = B(i, 2);
        cur_displacement = boundary_idx_dis_sorted(i, 2);
        cur_J = cur_dis_to_s_g_1 ^ n_s * cur_displacement ^ n;
        
        if cur_J > J_max
            J_max = cur_J;
            s_g_2_idx = cur_idx;
        end
    end
    
    S_g_idx = [s_g_1_idx, s_g_2_idx];
end