close all
clear
clc

%% load raw file
% wf:white foam
ptcloud_wf_initial = pcread('./Point Cloud/202206011815_without_normal.ply');
ptcloud_wf_deformed = pcread('./Point Cloud/202206011815_deformed_without_normal.ply');

%% extract object point cloud
row_indices_initial = GetPtCloudRowIndices(ptcloud_wf_initial.Location(:, 1));
row_indices_deformed = GetPtCloudRowIndices(ptcloud_wf_deformed.Location(:, 1));
DO_row_indices_initial = DetectDORowByColor(21500, 30000, row_indices_initial, [126, 131, 131], [142, 182, 216], ptcloud_wf_initial.Color);
DO_row_indices_deformed = DetectDORowByDepth(17000, 28000, row_indices_deformed, -0.4, ptcloud_wf_deformed.Location);
DO_pts_initial = GetBoundaryPts(DO_row_indices_initial, ptcloud_wf_initial.Location);
DO_pts_deformed = GetBoundaryPts(DO_row_indices_deformed, ptcloud_wf_deformed.Location);
ptcloud_DO_initial = ExtractObjectFromPtCloud(DO_row_indices_initial, ptcloud_wf_initial);
ptcloud_DO_deformed = ExtractObjectFromPtCloud(DO_row_indices_deformed(10:end, :), ptcloud_wf_deformed);
figure
pcshow(ptcloud_wf_deformed)
figure
pcshow(ptcloud_DO_deformed)
figure
plot(ptcloud_wf_deformed.Location(:, 3))

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


function DO_row_start_end_indices = DetectDORowByColor(start_idx_guess, end_idx_guess, row_indices, ...
                                                       backgraound_color, DO_color, ptcloud_color)
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
        
        color_log = zeros(1, pt_end_idx - pt_start_idx + 1);
        for pt_idx = pt_start_idx : pt_end_idx
            pt_color = [double(ptcloud_color(pt_idx, 1)), ... 
                        double(ptcloud_color(pt_idx, 2)), ... 
                        double(ptcloud_color(pt_idx, 3))];
            if sum(abs(pt_color - DO_color)) < sum(abs(pt_color - backgraound_color))
                color_log(1, pt_idx - pt_start_idx + 1) = 1;
            end
        end
        
        DO_start_idx = 0;
        for pt_idx = pt_start_idx + 1 : pt_end_idx - 5
            color_idx = pt_idx - pt_start_idx + 1;
            if color_log(1, color_idx) == 1 ...
              && color_log(1, color_idx - 1) == 0 ...
              && color_log(1, color_idx + 5) == 1
                DO_start_idx = pt_idx;
                break;
            end
        end
        DO_end_idx = 0;
        for pt_idx = pt_end_idx - 1 : -1 : pt_start_idx + 5
            color_idx = pt_idx - pt_start_idx + 1;
            if color_log(1, color_idx) == 1 ...
              && color_log(1, color_idx + 1) == 0 ...
              && color_log(1, color_idx - 5) == 1
                DO_end_idx = pt_idx;
                break;
            end
        end
        
        if DO_start_idx ~= 0 && DO_end_idx ~= 0
            DO_row_start_end_indices(DO_row_idx, :) = [DO_start_idx, DO_end_idx];
            DO_row_idx = DO_row_idx + 1;
        end
    end
    DO_row_start_end_indices = DO_row_start_end_indices(1 : DO_row_idx - 1, :);
end


function DO_pts = GetBoundaryPts(DO_row_indices, ptcloud_location)
    DO_pts = zeros(size(DO_row_indices, 1), size(DO_row_indices, 2), 3);
    for i = 1 : size(DO_row_indices, 1)
        row_start_idx = DO_row_indices(i, 1);
        row_end_idx = DO_row_indices(i, 2);
        for pt_idx = row_start_idx : row_end_idx
            DO_pts(i, pt_idx - row_start_idx + 1, :) = [ptcloud_location(pt_idx, 1), ... 
                                                        ptcloud_location(pt_idx, 2), ptcloud_location(pt_idx, 3)];
        end
    end
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