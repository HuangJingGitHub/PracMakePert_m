function passage_check_info = passageCheckAnalysis(obs_num, test_num)
    obs_dimension = [1, 1];  % obs length, width
    % obs_num = 20;
    obs_center = zeros(obs_num, 2);

    x_map_limit = [0, 50];
    y_map_limit = [0, 30];

    x_length = x_map_limit(1, 2) - x_map_limit(1, 1);
    y_length = y_map_limit(1, 2) - y_map_limit(1, 1);
    
    valid_passage_num = zeros(test_num, 1);
    valid_passage_visibility_num = zeros(test_num, 1);
    for test_idx = 1 : test_num
        for i = 1 : obs_num
           cur_x = rand();
           cur_y = rand();
           cur_x = cur_x * x_length;
           cur_y = cur_y * y_length;
           cur_center = [cur_x, cur_y];

           nearest_idx = knnsearch(obs_center, cur_center);
           dist_vec = cur_center - obs_center(nearest_idx, :);
           while abs(dist_vec(1, 1)) <= obs_dimension(1, 1) || abs(dist_vec(1, 2)) <= obs_dimension(1, 2)
               cur_x = rand();
               cur_y = rand();
               cur_x = cur_x * x_length;
               cur_y = cur_y * y_length;
               cur_center = [cur_x, cur_y];

               nearest_idx = knnsearch(obs_center, cur_center);
               dist_vec = cur_center - obs_center(nearest_idx, :);   
           end

           x_clearance = obs_dimension(1, 1) / 2;
           if cur_x < x_map_limit(1, 1) + x_clearance
               cur_x = x_map_limit(1, 1) + x_clearance;
           elseif cur_x > x_map_limit(1, 2) - x_clearance
               cur_x = x_map_limit(1, 2) - x_clearance;
           end

           y_clearance = obs_dimension(1, 2) / 2;
           if cur_y < y_map_limit(1, 1) + y_clearance
               cur_y = y_map_limit(1, 1) + y_clearance;
           elseif cur_y > y_map_limit(1, 2) - y_clearance
               cur_y = y_map_limit(1, 2) - y_clearance;
           end
           obs_center(i, :) = [cur_x, cur_y];

           obs_array(i) = polyshape([cur_x - x_clearance, cur_x + x_clearance, cur_x + x_clearance, cur_x - x_clearance],...
                                   [cur_y - y_clearance, cur_y - y_clearance, cur_y + y_clearance, cur_y + y_clearance]);
        end
        
        [valid_passage_pair, valid_passage_visibility_pair] = extendedVisibilityCheck(obs_array);
        valid_passage_num(test_idx, 1) = size(valid_passage_pair, 1);
        valid_passage_visibility_num(test_idx, 1) = size(valid_passage_visibility_pair, 1);
    end
    
    passage_check_info.x_map_limit = x_map_limit;
    passage_check_info.y_map_limit = y_map_limit;
    passage_check_info.obs_num = obs_num;
    passage_check_info.obs_dimension = obs_dimension;
    passage_check_info.test_num = test_num;
    passage_check_info.valid_passage_num = valid_passage_num;
    passage_check_info.valid_passage_visibility_num = valid_passage_visibility_num;
end

