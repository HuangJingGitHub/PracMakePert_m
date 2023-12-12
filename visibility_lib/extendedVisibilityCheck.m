function [valid_passage_pair, valid_passage_visibility_pair] = extendedVisibilityCheck(obs_array)
    addpath('./p_poly_dist')
    
    valid_passage_pair = [];
    valid_passage_visibility_pair = [];
    
    obs_num = size(obs_array, 2);
    for i = 1 : obs_num - 1
        for j = i + 1 : obs_num
            x_obs_i = obs_array(i).Vertices(:, 1).';
            y_obs_i = obs_array(i).Vertices(:, 2).';
            x_obs_j = obs_array(j).Vertices(:, 1).';
            y_obs_j = obs_array(j).Vertices(:, 2).';
            [d_min_i_j, x_min_i_j, y_min_i_j] = p_poly_dist(x_obs_i, y_obs_i, x_obs_j, y_obs_j);
            [d_min_j_i, x_min_j_i, y_min_j_i] = p_poly_dist(x_obs_j, y_obs_j, x_obs_i, y_obs_i);
            [d_min_val_i_j, d_min_idx_i_j] = min(d_min_i_j);
            [d_min_val_j_i, d_min_idx_j_i] = min(d_min_j_i);
            x_d_min_pair = zeros(1, 2);
            y_d_min_pair = zeros(1, 2);
            min_dist = 0;
            if d_min_val_i_j < d_min_val_j_i
                min_dist = d_min_val_i_j;
                x_d_min_pair = [x_obs_i(1, d_min_idx_i_j), x_min_i_j(d_min_idx_i_j, 1)];
                y_d_min_pair = [y_obs_i(1, d_min_idx_i_j), y_min_i_j(d_min_idx_i_j, 1)];
            else
                min_dist = d_min_val_j_i;
                x_d_min_pair = [x_obs_j(1, d_min_idx_j_i), x_min_j_i(d_min_idx_j_i, 1)];
                y_d_min_pair = [y_obs_j(1, d_min_idx_j_i), y_min_j_i(d_min_idx_j_i, 1)];              
            end
            passage_pt_1 = [x_d_min_pair(1, 1), y_d_min_pair(1, 1)];
            passage_pt_2 = [x_d_min_pair(1, 2), y_d_min_pair(1, 2)];
            passage_center = (passage_pt_1 + passage_pt_2) / 2;
            
            is_valid_passage = true;
            is_valid_passage_visibility = true;
            for k = 1 : obs_num
                if i == k || j == k
                    continue
                end
                
                if pureVisibilityCheck(passage_pt_1, passage_pt_2, obs_array(k)) == false
                    is_valid_passage_visibility = false;
                end
                
                if is_valid_passage_visibility == false
                    is_valid_passage = false;
                    break;
                end
                
                if is_valid_passage == true
                    x_obs_k = obs_array(k).Vertices(:, 1).';
                    y_obs_k = obs_array(k).Vertices(:, 2).';
                    [dist_to_passage_center, ~] = p_poly_dist(passage_center(1, 1), passage_center(1, 2),...
                                                     x_obs_k, y_obs_k);
                    obs_k_dist_to_passage_center = min(dist_to_passage_center);
                    if obs_k_dist_to_passage_center < min_dist / 2
                        is_valid_passage = false;
                    end
                end 
            end
            
            if is_valid_passage == true
                valid_passage_pair = [valid_passage_pair; i, j];
            end
            if is_valid_passage_visibility == true
                valid_passage_visibility_pair = [valid_passage_visibility_pair; i, j];
            end
        end
    end
end
