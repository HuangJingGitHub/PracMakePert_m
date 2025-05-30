clear all
close all
clc

data_directory = '/home/jing/Documents/Scripts/Passage_Traversing_Optimal_Path_Planning/src/ptopp/src/data/planning_time_3d/used';
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

    raw_data = zeros(30, 5);
    for col = 41:70
        str_data = raw_str_data{col, 1};
        split_str = split(str_data, ',');
        col_data = str2double(split_str)';
        raw_data(col - 40, :) = col_data;
    end

    trial_num = size(raw_data, 1);
    len_time = raw_data(:, 1);
    interior_time = raw_data(:, 2);
    max_clearance_time = raw_data(:, 3);
    ptopp_time = raw_data(:, 4);
    ptopp_all_time = raw_data(:, 5);

    %% Exclude data labelled 1e+06   
    len_time = FilterData(len_time);
    interior_time = FilterData(interior_time);
    max_clearance_time = FilterData(max_clearance_time);
    ptopp_time = FilterData(ptopp_time);
    ptopp_all_time = FilterData(ptopp_all_time);

    res_row = obs_num / obs_num_step;
    prms_res(res_row, 1) = obs_num;
    prms_res(res_row, 2) = mean(len_time);
    prms_res(res_row, 3) = mean(interior_time);
    prms_res(res_row, 4) = mean(max_clearance_time);
    prms_res(res_row, 5) = mean(ptopp_time);
    prms_res(res_row, 6) = mean(ptopp_all_time);      
end


function filter_res = FilterData(raw_data)
    length = size(raw_data, 1);
    filter_res = raw_data;

    idx = 1;
    for i = 1 : length
        if filter_res(i, 1) < 1e5
            filter_res(idx, 1) = filter_res(i, 1);
            idx = idx + 1;
        end
    end
    filter_res = filter_res(1 : idx - 1, 1);
end