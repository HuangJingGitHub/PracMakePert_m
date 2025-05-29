clear all
close all
clc

data_directory = '/home/jing/Documents/Scripts/Passage_Traversing_Optimal_Path_Planning/src/ptopp/src/data/planning_time_2d/used';
file_list = dir(data_directory);

obs_num_step = 20;
obs_num_list = 20:obs_num_step:200;
obs_num_list = obs_num_list';
obs_num_variations = size(obs_num_list, 1);
rrts_res = zeros(obs_num_variations, 6);
prms_res = zeros(obs_num_variations, 6);

trial_num = 30;
%% Raw data
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
    
    raw_data = zeros(30, 5);
    for col = 10:39
        str_data = raw_str_data{col, 1};
        split_str = split(str_data, ',');
        col_data = str2double(split_str)';
        raw_data(col - 9, :) = col_data;
    end

    trial_num = size(raw_data, 1);
    len_time = raw_data(:, 1);
    interior_time = raw_data(:, 2);
    max_clearance_time = raw_data(:, 3);
    ptopp_time = raw_data(:, 4);
    ptopp_all_time = raw_data(:, 5);

    res_row = obs_num / obs_num_step;
    rrts_res(res_row, 1) = obs_num;
    rrts_res(res_row, 2) = sum(len_time) / trial_num;
    rrts_res(res_row, 3) = sum(interior_time) / trial_num;
    rrts_res(res_row, 4) = sum(max_clearance_time) / trial_num;
    rrts_res(res_row, 5) = sum(ptopp_time) / trial_num;
    rrts_res(res_row, 6) = sum(ptopp_all_time) / trial_num;  
end

