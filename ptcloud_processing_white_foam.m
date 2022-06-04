close all
clear
clc

%% load raw file
% wf:white foam
ptcloud_wf_initial = pcread('./Point Cloud/202206011815_without_normal.ply');
ptcloud_wf_deformed = pcread('./Point Cloud/202206011815_deformed_without_normal.ply');

%% extract object point cloud
pt_num_initial = ptcloud_wf_initial.Count;
pt_num_deformed = ptcloud_wf_deformed.Count;
row_indices_initial = GetPtCloudRowIndices(ptcloud_wf_initial.Location(:, 1));
row_indices_deformed = GetPtCloudRowIndices(ptcloud_wf_deformed.Location(:, 1));
DO_row_indices_initial = DetectDORowByColor(21500, 30000, row_indices_initial, [126, 131, 131], [142, 182, 216], ptcloud_wf_initial.Color);
DO_row_indices_deformed = DetectDORowByColor(16000, 30000, row_indices_initial, [126, 131, 131], [142, 182, 216], ptcloud_wf_deformed.Color);
DO_pts_initial = GetBoundaryPts(DO_row_indices_initial, ptcloud_wf_initial.Location);


function row_start_end_indices = GetPtCloudRowIndices(x_coordinates)
    pt_num = size(x_coordinates, 1);
    row_start_end_indices = zeros(500, 2);
    row_start_end_indices(1, 1) = 1;
    row_idx = 1;
    i = 5;
    while i < pt_num
        if x_coordinates(i, 1) > x_coordinates(i - 1) ...
          && x_coordinates(i, 1) > x_coordinates(i + 1, 1)
            row_start_end_indices(row_idx, 2) = i;
            row_start_end_indices(row_idx + 1, 1) = i + 1;
            row_idx = row_idx + 1;
            i = i + 5;
        else
            i = i + 1;
        end
    end
    row_start_end_indices(row_idx, 2) = pt_num;
    row_start_end_indices = row_start_end_indices(1:row_idx, :);   
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