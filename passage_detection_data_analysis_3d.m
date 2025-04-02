clear all
close all
clc

data_directory = ['/home/jing/Documents/Scripts/Passage_Traversing_Optimal_Path_Planning' ...
                   '/src/ptopp/src/data/passage_detection_3d'];
file_list = dir(data_directory);

obs_num_step = 20;
obs_num_list = 20:obs_num_step:200;
obs_num_list = obs_num_list';
obs_num_types = size(obs_num_list, 1);
res = zeros(obs_num_types, 10); 

trial_num = 50;
%% Raw data
%% 1-3d passage numbers-2-base passage numbers-3-base cell numbers
for i = 1 : length(file_list)
    file = file_list(i, 1);
    if file.isdir
        continue
    end
    if strcmp(file.name(:, end - 2 : end), 'txt') == 0
        continue
    end
    raw_str_data = importdata(file.name, '\n');
    obs_num_str = raw_str_data{2}(1, end - 2 : end);
    obs_num = str2num(obs_num_str);
    if mod(obs_num, obs_num_step) ~= 0
        continue
    end
    vaying_obs_side_str = raw_str_data{5}(1, end - 4 : end);
    if strcmp(vaying_obs_side_str, ' true') == 0
        continue
    end

    raw_data = zeros(50, 3);
    for col = 8:57
        str_data = raw_str_data{col, 1};
        split_str = split(str_data, ',');
        col_data = str2double(split_str)';
        raw_data(col - 7, :) = col_data;
    end
    passage_num_3d = raw_data(:, 1);
    passage_num_2d = raw_data(:, 2);
    cell_num_2d = raw_data(:, 3);  
    
    raw_data = zeros(50, 3);
    for col = 59:108
        str_data = raw_str_data{col, 1};
        split_str = split(str_data, ',');
        col_data = str2double(split_str)';
        raw_data(col - 7, :) = col_data;
    end
    psg_detection_time_3d = raw_data(:, 1);
    psg_detection_time_2d = raw_data(:, 2);
    cell_detection_time_2d = raw_data(:, 3);      
    % res row format: [1-obstacle number, 2-average 3d passage number, 
    %                  3-standard deviation of 3d passage number,
    %                  4-average passage number 2d, 5-standard deviation of passage numbes 2d,
    %                  6-average cell number 2d, 7-standard deviation of cell numbers 2d,
    %                  8-average 3d passage detection time, 9-average 2d passage detection time, 10-average cell detection time]
    res_row = obs_num / obs_num_step;
    res(res_row, 1) = obs_num;
    res(res_row, 2) = sum(passage_num_3d) / trial_num;
    res(res_row, 3) = std(passage_num_3d);
    res(res_row, 4) = sum(passage_num_2d) / trial_num;
    res(res_row, 5) = std(passage_num_2d);
    res(res_row, 6) = sum(cell_num_2d) / trial_num;
    res(res_row, 7) = std(cell_num_2d);    
    res(res_row, 8) = sum(psg_detection_time_3d) / trial_num;
    res(res_row, 9) = sum(psg_detection_time_2d) / trial_num;
    res(res_row, 10) = sum(cell_detection_time_2d) / trial_num;
end


%%
fig_1 = figure('Position', [200, 200, 920, 570]);
bar_1 = bar(obs_num_list, [res(:, 2), res(:, 4), res(:, 6)], 'BarWidth', 1);
% colororder('earth')
hold on
err_1 = errorbar(obs_num_list - 4.5, res(:, 2), res(:, 3), res(:, 3));
err_1.Color = [0, 0, 0]; 
err_1.LineStyle = 'none';
err_1.LineWidth = 1;
err_2 = errorbar(obs_num_list, res(:, 4), res(:, 5), res(:, 5));
err_2.Color = [0, 0, 0]; 
err_2.LineStyle = 'none';
err_2.LineWidth = 1;
err_3 = errorbar(obs_num_list + 4.5, res(:, 6), res(:, 7), res(:, 7));
err_3.Color = [0, 0, 0]; 
err_3.LineStyle = 'none';
err_3.LineWidth = 1;

set(gca, 'FontSize', 16)
ylabel('Passage/Cell number', 'FontName', 'Arial', 'FontSize', 18);
xlabel('Obstacle number', 'FontName', 'Arial', 'FontSize', 18);
leg_1 = legend({'3D passages', 'Passages on the base', 'Cells on the base'},'FontName', 'Arial', 'FontSize', 18, 'Location', 'northwest');
                %'Position', [0.18 0.8067 0.3576 0.0991]);
ax = gca;
ax.XGrid = 'off';
ax.YGrid = 'on';
ylim([0, 800])
set(gcf, 'Renderer', 'Painters')
% Figure size may be changed in plotting. Use drawnow to update figure and pause to let operations finish.
drawnow
pause(0.1)
fig_1.Position = [200, 200, 920, 570];
%print(fig_1, '../../img/Passage_Cell_Num_3D', '-depsc')

%%
fig_2 = figure('Position', [200, 200, 920, 570]);
plot(res(:, 1), res(:, 8), 'LineWidth', 2, 'Marker','^', 'MarkerSize', 10)
hold on
plot(res(:, 1), res(:, 9), 'LineWidth', 2, 'Marker','x', 'MarkerSize', 10)

set(gca, 'FontSize', 16)
ylabel('Detection time (ms)', 'FontName', 'Arial', 'FontSize', 18);
xlabel('Obstacle number', 'FontName', 'Arial', 'FontSize', 18);
leg_1 = legend({'3D passage detection', 'Planar passage detection'},'FontName', 'Arial', 'FontSize', 18, 'Location', 'northwest');
ax = gca;
ax.XGrid = 'off';
ax.YGrid = 'on';
xlim([0, 205])
ax.XTick = 0:20:200;
ylim([0, 220])
%ax.YTick = 0:200:1000;
set(gcf, 'Renderer', 'Painters')
drawnow
pause(0.1)
fig_2.Position = [400, 400, 920, 570];
print(fig_2, '../../img/Passage_Detection_Time_3D', '-depsc')