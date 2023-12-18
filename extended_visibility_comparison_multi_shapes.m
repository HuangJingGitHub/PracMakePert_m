clear all
close all
clc

addpath('./visibility_lib/')

obs_num_list = 10 : 10 : 100;
obs_num_list = obs_num_list.';
test_num = 10;
is_data_store_needed = false;

loop_num = length(obs_num_list);
for i = 1 : loop_num
    if is_data_store_needed == false
        break
    end
    i
    obs_num = obs_num_list(i, 1);
    passage_check_info_list(i) = passageCheckMultiShapes(obs_num, test_num);
end

if is_data_store_needed
    cur_time = datetime('now','TimeZone','local','Format','d-MMM-y HH:mm:ss Z');
    data_str = datestr(cur_time);
    mat_file_name = append(data_str, ' passage_check_various_shapes.mat');
    save_path = './Saved Data/';
    save_path = append(save_path, mat_file_name);
    save_path = strrep(save_path, ':', '-');
    
    save(save_path, 'passage_check_info_list');
end


if is_data_store_needed == false
    load('./Saved Data/13-Dec-2023 14-41-31 passage_check_various_shapes.mat')
    obs_num_list = zeros(size(passage_check_info_list));
end

valid_passage_avg_deviation = zeros(loop_num, 2);
valid_passage_visibility_avg_deviation = zeros(loop_num, 2);
for i = 1 : loop_num
    obs_num_list(1, i) = passage_check_info_list(i).obs_num;
    cur_valid_passage_num_list = passage_check_info_list(i).valid_passage_num;
    cur_valid_passage_visibility_list = passage_check_info_list(i).valid_passage_visibility_num;
    
    valid_passage_avg_deviation(i, 1) = mean(cur_valid_passage_num_list);
    valid_passage_avg_deviation(i, 2) = std(cur_valid_passage_num_list);
    valid_passage_visibility_avg_deviation(i, 1) = mean(cur_valid_passage_visibility_list);
    valid_passage_visibility_avg_deviation(i, 2) = std(cur_valid_passage_visibility_list);
end

%%
obs_num_var = [ones(length(obs_num_list), 1), obs_num_list'];
passage_num_var = valid_passage_avg_deviation(:, 1);
linear_coe = obs_num_var \ passage_num_var;
passage_num_cal = obs_num_var * linear_coe;
R_sqr = 1 - sum((passage_num_var - passage_num_cal).^2) / sum((passage_num_var - mean(passage_num_var)).^2);

passage_num_var = valid_passage_visibility_avg_deviation(:, 1);
linear_coe = obs_num_var \ passage_num_var;
passage_num_cal = obs_num_var * linear_coe;
R_sqr_visibility = 1 - sum((passage_num_var - passage_num_cal).^2) / sum((passage_num_var - mean(passage_num_var)).^2);

%%
fig_1 = figure('Position', [200, 200, 920, 570]);
bar_1 = bar(obs_num_list, [valid_passage_avg_deviation(:, 1), valid_passage_visibility_avg_deviation(:, 1)], 'BarWidth', 1.2);
hold on
err_1 = errorbar(obs_num_list - 1.8, valid_passage_avg_deviation(:, 1), valid_passage_avg_deviation(:, 2), ...
                valid_passage_avg_deviation(:, 2));
err_1.Color = [0, 0, 0]; 
err_1.LineStyle = 'none';
err_1.LineWidth = 1;
err_2 = errorbar(obs_num_list + 1.5, valid_passage_visibility_avg_deviation(:, 1), valid_passage_visibility_avg_deviation(:, 2), ...
                valid_passage_visibility_avg_deviation(:, 2));
err_2.Color = [0, 0, 0]; 
err_2.LineStyle = 'none';
err_2.LineWidth = 1;

set(gca, 'FontSize', 15)
ylabel('Passage number', 'FontName', 'Arial', 'FontSize', 17);
xlabel('Obstacle number', 'FontName', 'Arial', 'FontSize', 17);
leg_1 = legend({'Visibility condition', 'Extended visibility condition'},'FontName', 'Arial', 'FontSize', 17, 'Location', 'northwest');
                %'Position', [0.18 0.8067 0.3576 0.0991]);
ax = gca;
ax.XGrid = 'off';
ax.YGrid = 'on';
%ylim([0, 1500])
set(gcf, 'Renderer', 'Painters')
% print(fig_1, './Figures/Passage_Number_Obs_Num_Multi_Shapes_Dims_12_13', '-depsc')