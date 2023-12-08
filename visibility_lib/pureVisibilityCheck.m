function valid_passage = pureVisibilityCheck(passage_pt_1, passage_pt_2, obs_obj)
    valid_passage = true;
    obs_vertex_num = size(obs_obj.Vertices, 1);
    for i = 1 : obs_vertex_num - 1
        if segmentsIntersectionCheck(passage_pt_1, passage_pt_2, ...
                                     obs_obj.Vertices(i, :), obs_obj.Vertices(i + 1, :)) == true
           valid_passage = false;
        end
    end
    if segmentsIntersectionCheck(passage_pt_1, passage_pt_2, ...
                                obs_obj.Vertices(end, :), obs_obj.Vertices(1, :)) == true
       valid_passage = false;
    end
end