clear all
close all
clc

data_directory = '/home/jing/Documents/Scripts/Passage_Traversing_Optimal_Path_Planning/src/ptopp/src/data';
file_list = dir(data_directory);

obs_num_step = 20;
obs_num_list = 20:obs_num_step:200;
obs_num_list = obs_num_list';
obs_num_variations = size(obs_num_list, 1);
res = zeros(obs_num_variations, 9); 

trial_num = 50;
%% Raw data
%% 1-passage number using brute-force traversal-2-passage number detected using Delaunay graph
%% -3-valid passage segments number in extended visibility check-4-cell number in Gabriel condition
%% -5-cell number from valid passage segments
%{
for i = 1 : length(file_list)
    file = file_list(i, 1);
    if length(file.name) < 3
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
    if strcmp(vaying_obs_side_str, 'false') == 0
        continue
    end
    
    raw_data = zeros(50, 5);
    for col = 9:58
        str_data = raw_str_data{col, 1};
        split_str = split(str_data, ',');
        col_data = str2double(split_str)';
        raw_data(col - 8, :) = col_data;
    end

    trial_num = size(raw_data, 1);
    passage_num = raw_data(:, 2);
    passage_segment_num = raw_data(:, 3);
    cell_num = raw_data(:, 4);
    cell_num_from_psg_seg = raw_data(:, 5);
    
    % res row format: [1-obstacle number, 2-average passage number using Gabriel condition, 
    %                  3-standard deviation of passage numbers using Gabriel condition,
    %                  4-average passage number using passage segmetns only, 5-standard deviation of passage numbes using passage segments,
    %                  6-average cell number using Gabriel condition, 7-standard deviation of cell numbers using Gabriel condition,
    %                  8-average cell number using passage segmetns only, 9-standard deviation of cell numbers using passga segments]
    res_row = obs_num / obs_num_step;
    res(res_row, 1) = obs_num;
    res(res_row, 2) = sum(passage_num) / trial_num;
    res(res_row, 3) = std(passage_num);
    res(res_row, 4) = sum(passage_segment_num) / trial_num;
    res(res_row, 5) = std(passage_segment_num);
    res(res_row, 6) = sum(cell_num) / trial_num;
    res(res_row, 7) = std(cell_num);    
    res(res_row, 8) = sum(cell_num_from_psg_seg) / trial_num;
    res(res_row, 9) = std(cell_num_from_psg_seg);
end
%}

load('passage_data_analyszed_20250121.mat')
%%
fig_1 = figure('Position', [200, 200, 920, 570]);
bar_1 = bar(obs_num_list, [res(:, 2), res(:, 4)], 'BarWidth', 1.2);
hold on
err_1 = errorbar(obs_num_list - 3.5, res(:, 2), res(:, 3), res(:, 3));
err_1.Color = [0, 0, 0]; 
err_1.LineStyle = 'none';
err_1.LineWidth = 1;
err_2 = errorbar(obs_num_list + 2.9, res(:, 4), res(:, 5), res(:, 5));
err_2.Color = [0, 0, 0]; 
err_2.LineStyle = 'none';
err_2.LineWidth = 1;

set(gca, 'FontSize', 16)
ylabel('Passage number', 'FontName', 'Arial', 'FontSize', 18);
xlabel('Obstacle number', 'FontName', 'Arial', 'FontSize', 18);
leg_1 = legend({'Gabriel condition', 'Only passage segments'},'FontName', 'Arial', 'FontSize', 18, 'Location', 'northwest');
                %'Position', [0.18 0.8067 0.3576 0.0991]);
ax = gca;
ax.XGrid = 'off';
ax.YGrid = 'on';
ylim([0, 500])
set(gcf, 'Renderer', 'Painters')
%print(fig_1, '../img/Passage_Num_Obs_Num_40_0121', '-depsc')
print(fig_1, '../img/Passage_Num_Obs_Num_Varying_Size_0121', '-depsc')

%%
fig_2 = figure('Position', [400, 400, 920, 570]);
bar_2 = bar(obs_num_list, [res(:, 6), res(:, 8)], 'BarWidth', 1.2);
hold on
err_2_1 = errorbar(obs_num_list - 3.5, res(:, 6), res(:, 7), res(:, 7));
err_2_1.Color = [0, 0, 0]; 
err_2_1.LineStyle = 'none';
err_2_1.LineWidth = 1;
err_2_2 = errorbar(obs_num_list + 2.9, res(:, 8), res(:, 9), res(:, 9));
err_2_2.Color = [0, 0, 0]; 
err_2_2.LineStyle = 'none';
err_2_2.LineWidth = 1;

set(gca, 'FontSize', 16)
ylabel('Cell number', 'FontName', 'Arial', 'FontSize', 18);
xlabel('Obstacle number', 'FontName', 'Arial', 'FontSize', 18);
leg_2 = legend({'Gabriel condition', 'Only passage segments'},'FontName', 'Arial', 'FontSize', 18, 'Location', 'northwest');
                %'Position', [0.18 0.8067 0.3576 0.0991]);
ax = gca;
ax.XGrid = 'off';
ax.YGrid = 'on';
ylim([0, 500])
set(gcf, 'Renderer', 'Painters')
%print(fig_2, '../img/Cell_Num_Obs_Num_40_0121', '-depsc')
print(fig_2, '../img/Cell_Num_Obs_Num_Varying_Size_0121', '-depsc')