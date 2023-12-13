function passage_check_info = passageCheckMultiShapesDimensions(obs_num, test_num, obs_dimension)
    % obs_dimension = [1, 1];  % obs length, width
    obs_type_num = 3;
    obs_center = zeros(obs_num, 2);
    x_clearance = obs_dimension(1, 1) / 2;
    y_clearance = obs_dimension(1, 2) / 2;
    std_obs_vertices{1, 1} = [-0.5, 0.5, 0.5, -0.5;
                              -0.5, -0.5, 0.5, 0.5] * obs_dimension(1, 1);
    std_obs_vertices{1, 2} = [-0.25, 0.25, 0.25, -0.25;
                              -0.5, -0.5, 0.5, 0.5] * obs_dimension(1, 1);   
    std_obs_vertices{1, 3} = [-0.5, 0.5, 0;
                              -sqrt(3) / 6, -sqrt(3) / 6, sqrt(3) / 3] * obs_dimension(1, 1);

    x_map_limit = [0, 50];
    y_map_limit = [0, 30];

    x_length = x_map_limit(1, 2) - x_map_limit(1, 1);
    y_length = y_map_limit(1, 2) - y_map_limit(1, 1);
    
    valid_passage_num = zeros(test_num, 1);
    valid_passage_visibility_num = zeros(test_num, 1);
    for test_idx = 1 : test_num
        obs_type = randi([1, obs_type_num], obs_num, 1);
        for i = 1 : obs_num
           cur_x = rand();
           cur_y = rand();
           cur_x = cur_x * x_length;
           cur_y = cur_y * y_length;
           cur_center = [cur_x, cur_y];

           nearest_idx = knnsearch(obs_center, cur_center);
           dist_vec = cur_center - obs_center(nearest_idx, :);
           adjust_ratio = 1;
           if obs_dimension(1, 1) == 5
               adjust_ratio = 0.85;
           end
           while i > 1 && (abs(dist_vec(1, 1)) <= obs_dimension(1, 1) * adjust_ratio||...
                           abs(dist_vec(1, 2)) <= obs_dimension(1, 2) * adjust_ratio)
               cur_x = rand();
               cur_y = rand();
               cur_x = cur_x * x_length;
               cur_y = cur_y * y_length;
               cur_center = [cur_x, cur_y];

               nearest_idx = knnsearch(obs_center, cur_center);
               dist_vec = cur_center - obs_center(nearest_idx, :);   
           end

           if cur_x < x_map_limit(1, 1) + x_clearance
               cur_x = x_map_limit(1, 1) + x_clearance * 1.35;
           elseif cur_x > x_map_limit(1, 2) - x_clearance
               cur_x = x_map_limit(1, 2) - x_clearance * 1.35;
           end
      
           if cur_y < y_map_limit(1, 1) + y_clearance
               cur_y = y_map_limit(1, 1) + y_clearance * 1.35;
           elseif cur_y > y_map_limit(1, 2) - y_clearance
               cur_y = y_map_limit(1, 2) - y_clearance * 1.35;
           end
           obs_center(i, :) = [cur_x, cur_y];

           cur_rotate_angle = rand() * 360;
           cur_rotz = rotz(cur_rotate_angle);
           cur_rotz = cur_rotz(1:2, 1:2);
           rotated_obs_pos = cur_rotz * std_obs_vertices{1, obs_type(i, 1)};
           rotated_obs_pos(1, :) = rotated_obs_pos(1, :) + cur_x;
           rotated_obs_pos(2, :) = rotated_obs_pos(2, :) + cur_y;
           
           obs_array(i) = polyshape(rotated_obs_pos(1, :), rotated_obs_pos(2,:));
        end
        
        [valid_passage_pair, valid_passage_visibility_pair] = extendedVisibilityCheck(obs_array);
        valid_passage_num(test_idx, 1) = length(valid_passage_pair);
        valid_passage_visibility_num(test_idx, 1) = length(valid_passage_visibility_pair);
    end
    
    passage_check_info.x_map_limit = x_map_limit;
    passage_check_info.y_map_limit = y_map_limit;
    passage_check_info.obs_num = obs_num;
    passage_check_info.obs_dimension = obs_dimension;
    passage_check_info.test_num = test_num;
    passage_check_info.valid_passage_num = valid_passage_num;
    passage_check_info.valid_passage_visibility_num = valid_passage_visibility_num;
end
