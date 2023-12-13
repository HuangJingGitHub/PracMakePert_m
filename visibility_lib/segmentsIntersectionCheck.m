%% 
function is_intersected = segmentsIntersectionCheck(p_1, p_2, q_1, q_2) 
    is_intersected = false;

    orientation_1 = orientationType(p_1, p_2, q_1);
    orientation_2 = orientationType(p_1, p_2, q_2);
    orientation_3 = orientationType(q_1, q_2, p_1);
    orientation_4 = orientationType(q_1, q_2, p_2);
    
    if orientation_1 ~= orientation_2 && orientation_3 ~= orientation_4
        is_intersected = true;
    end
    
    if orientation_1 == 0 && onSegmentCheck(p_1, p_2, q_1)
        is_intersected = true;
    elseif orientation_2 == 0 && onSegmentCheck(p_1, p_2, q_2)
        is_intersected = true;
    elseif orientation_3 == 0 && onSegmentCheck(q_1, q_2, p_1)
        is_intersected = true;
    elseif orientation_4 == 0 && onSegmentCheck(q_1, q_2, p_2)
        is_intersected = true;
    end   
end

%%
function orientation_type = orientationType(p_1, p_2, r) 
    % o---------------o
    % p_1--------------p_2 (1x2 vector)
    orientation_type = 0;
    
    segment_vec_1 = (p_2 - p_1).';
    segment_vec_2 = (r - p_1).';
    if isequal(size(segment_vec_1), [2, 1]) == false || isequal(size(segment_vec_2), [2, 1]) == false
        error('Wrong dimension of input points')
    end
    det_val = det([segment_vec_1, segment_vec_2]);
    if abs(det_val) < 1e-7
        orientation_type = 0;
    elseif det_val > 0
        orientation_type = 1;
    else
        orientation_type = 2;
    end
end

%%
function is_on_segment = onSegmentCheck(p_1, p_2, r)
    % p_1, p_2, r are conlinear, check if r is on the segment of p_1---p_2
    is_on_segment = false;
    if r(1) >= min(p_1(1), p_2(1)) && r(1) <= max(p_1(1), p_2(1)) && ...
       r(2) >= min(p_1(2), p_2(2)) && r(2) <= max(p_1(2), p_2(2)) 
        is_on_segment = true;
    end
end
