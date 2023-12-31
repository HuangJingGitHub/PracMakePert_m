#ifndef PATH_SMOOTHING_INCLUDED
#define PATH_SMOOTHING_INCLUDED

#include <eigen3/Eigen/Dense>
#include "RRTStar_DOM_kd_tree.hpp"

using namespace Eigen;

vector<Point2f> QuadraticBSplineSmoothing(const vector<RRTStarNode*>& node_path) {
    vector<Point2f> res;
    if (node_path.size() < 3) {
        cout << "No sufficient control points on the input path!\n"
             << "(At least 3 control points are needed for quadratic B-Spline smoothing.)\n";
        return res;
    }

    const float step_width = 0.05;   // For MatrixxXf.block() mathod needs constexpr as argument.
    const int step_num = 1 / step_width + 1,
              pts_num = node_path.size();

    Matrix3f coefficient_mat_1, coefficient_mat_2, coefficient_mat_3;
    MatrixXf parameter_var(3, step_num), 
             control_pts(2, pts_num);
    Matrix<float, 2, Dynamic> smoothed_path;

    coefficient_mat_1 << 1, -2, 1, -1.5, 2, 0, 0.5, 0, 0;
    coefficient_mat_2 << 0.5, -1, 0.5, -1, 1, 0.5, 0.5, 0, 0;
    coefficient_mat_3 << 0.5, -1, 0.5, -1.5, 1, 0.5, 1, 0, 0;
    
    for (int i = 0; i < step_num; i++) {
        float cur_var = step_width * i;
        parameter_var(0, i) = cur_var * cur_var;
        parameter_var(1, i) = cur_var;
        parameter_var(2, i) = 1;
    }
    for (int i = 0; i < pts_num; i++) {
        control_pts(0, i) = node_path[i]->pos.x;
        control_pts(1, i) = node_path[i]->pos.y;
    }

    MatrixXf cur_pts = control_pts.block<2, 3>(0, 0);
    MatrixXf cur_spline = cur_pts * coefficient_mat_1 * parameter_var;
    smoothed_path = cur_spline;

    for (int i = 1; i < pts_num - 3; i++) {
        cur_pts = control_pts.block<2, 3>(0, i);
        cur_spline = cur_pts * coefficient_mat_2 * parameter_var;
        smoothed_path.conservativeResize(2, smoothed_path.cols() + cur_spline.cols());
        smoothed_path.block<2, step_num>(0, smoothed_path.cols() - step_num) = cur_spline;
    }
    cur_pts = control_pts.block<2, 3>(0, pts_num - 3);
    cur_spline = cur_pts * coefficient_mat_3 * parameter_var;
    smoothed_path.conservativeResize(2, smoothed_path.cols() + cur_spline.cols());
    smoothed_path.block<2, step_num>(0, smoothed_path.cols() - step_num) = cur_spline;

/*     cout << "path smoothing:\n"
         << node_path.size() << ' '
         << smoothed_path.rows() << " x " << smoothed_path.cols() << '\n'
         << cur_spline.rows() << " x " << cur_spline.cols() << endl; */
    for (int i = 0; i < smoothed_path.cols(); i++)
        res.push_back(Point2f(smoothed_path(0, i), smoothed_path(1, i)));
    return res;
}


int SearchByDistance(vector<Point2f>& search_path, Point2f desired_pos) {
    int res_idx = 0;
    if (search_path.empty()) {
        cout << "An empty path is given." << endl;
        return res_idx;
    }

    Point2f trivial_sol = search_path.back();
    float ref_distance = norm(desired_pos - trivial_sol),
          cur_distance_dif;

    for (int i = search_path.size() - 1; i >= 0; i--) {
        cur_distance_dif = abs(norm(desired_pos - search_path[i]) - ref_distance);
        if (cur_distance_dif < 2)
            res_idx = i;
    }
    return res_idx;
}


vector<vector<Point2f>> GeneratePathSet(const vector<Point2f>& initial_feedback_pts, 
                                        const vector<Point2f>& target_feedback_pts, 
                                        int pivot_idx,
                                        float feedback_pts_radius,
                                        const vector<PolygonObstacle>& obs,
                                        Mat source_img) {
    vector<vector<Point2f>> res_path_set(initial_feedback_pts.size());                                            
    RRTStarPlanner planner(initial_feedback_pts[pivot_idx], target_feedback_pts[pivot_idx], obs);
    bool plan_success = planner.Plan(source_img, feedback_pts_radius, true);
    if (!plan_success) {
        cout << "Path planning failed! An empty path set is reutrned.\n";
        return {};
    }

    vector<RRTStarNode*> pivot_path = planner.GetPath();
    vector<RRTStarNode*> sparse_pivot_path;
    for (int i = 0; i < pivot_path.size(); i += 1)
        sparse_pivot_path.push_back(pivot_path[i]);
    if (sparse_pivot_path.back() != pivot_path.back())
        sparse_pivot_path.push_back(pivot_path.back());
    vector<Point2f> smooth_pivot_path = QuadraticBSplineSmoothing(sparse_pivot_path);
    res_path_set[pivot_idx] = smooth_pivot_path;
    
    for (int i = 0; i < initial_feedback_pts.size(); i++) {
        if (i != pivot_idx) {
            Point2f cur_dif = initial_feedback_pts[i] - initial_feedback_pts[pivot_idx];
            res_path_set[i] = smooth_pivot_path;
            for (Point2f& pt : res_path_set[i])
                pt += cur_dif;
            
            if (normSqr(res_path_set[i].back() - target_feedback_pts[i]) > normSqr(res_path_set[i].front() - target_feedback_pts[i])){
                res_path_set[i].erase(res_path_set[i].begin() + 1, res_path_set[i].end());
            }
            else {
                int truncation_idx = SearchByDistance(res_path_set[i], target_feedback_pts[i]);
                res_path_set[i].erase(res_path_set[i].begin() + truncation_idx, res_path_set[i].end());
            }
            
            Point2f temp_end = res_path_set[i].back(), segment = target_feedback_pts[i] - temp_end;
            float segment_length = cv::norm(segment), temp_path_length = cv::norm(temp_end - res_path_set[i].front()), 
                  interpolation_ratio;
            int segment_pt_num;
            if (temp_path_length > 1)  // in case temp_path_length becomes 0
                segment_pt_num = segment_length / temp_path_length * res_path_set[i].size();
            else
                segment_pt_num = 100;
            for (int interpolation_idx = 1; interpolation_idx <= segment_pt_num; interpolation_idx++) {
                interpolation_ratio = (float)interpolation_idx / segment_pt_num;
                res_path_set[i].push_back(temp_end + segment * interpolation_ratio);
            }
            // res_path_set[i].push_back(target_feedback_pts[i]);
        }
    }
    return res_path_set; 
}


vector<Point2f> ProcessCollisionPath(const vector<Point2f>& pivot_path, 
                                    Point2f pivot_pt, Point2f cur_pt, 
                                    Point2f cur_target_pt,
                                    const PolygonObstacle& original_obstacle, 
                                    float safety_dis = 20) {
    vector<Point2f> inflated_obs_vertices = original_obstacle.vertices;
    Point2f vertices_centroid = Point2f(0, 0);
    for (Point2f& pt : inflated_obs_vertices)
        vertices_centroid += pt;
    vertices_centroid /= (float) inflated_obs_vertices.size();
    for (Point2f& pt : inflated_obs_vertices)
        pt = vertices_centroid + 1.1 * (pt - vertices_centroid);
    PolygonObstacle obstacle(inflated_obs_vertices);


    vector<Point2f> res_path(pivot_path.size());
    Point2f cur_dif = cur_pt - pivot_pt;
    float cur_dif_norm = cv::norm(cur_dif);
    vector<int> collision_indices;
    
    for (int i = 0; i < pivot_path.size(); i++) {
        Point2f ref_pt = pivot_path[i] + cur_dif;
        if (!ObstacleFree(obstacle, pivot_path[i], ref_pt))
            collision_indices.push_back(i);
    }
    
    // std::sort(collision_indices.begin(), collision_indices.end());
    int collision_start_idx = collision_indices.front(), 
        collision_end_idx = collision_indices.back(),
        reconnection_start_idx = collision_start_idx, reconnection_end_idx = collision_end_idx;
    float dis_ref = 3 * safety_dis;
    for (int i = collision_start_idx - 1; i >= 0; i--) {
        float dis_to_obs = MinDistanceToObstacle(obstacle, pivot_path[i] + cur_dif);
        if (dis_to_obs >= dis_ref) {
            reconnection_start_idx = i;
            break;
        }
    }
    if (reconnection_start_idx == collision_start_idx)
        reconnection_start_idx = 0;
    
    Point2f intersection_start_pt = GetClosestIntersectionPt(obstacle, pivot_path[collision_start_idx],
                            pivot_path[collision_start_idx] + cur_dif, pivot_path[collision_start_idx]),
            shift_pt_1 = intersection_start_pt - safety_dis / cur_dif_norm * cur_dif 
                         - (pivot_path[reconnection_start_idx] + cur_dif); 
    for (int i = collision_end_idx + 1; i < pivot_path.size(); i++) {
        float dis_to_obs = MinDistanceToObstacle(obstacle, pivot_path[i] + cur_dif);
        if (dis_to_obs >= dis_ref) {
            reconnection_end_idx = i;
            break;
        }
    }
    if (reconnection_end_idx == collision_end_idx)
        reconnection_end_idx = pivot_path.size() - 1;
    Point2f intersection_end_pt = GetClosestIntersectionPt(obstacle, pivot_path[collision_end_idx],
                            pivot_path[collision_end_idx] + cur_dif, pivot_path[collision_end_idx]),
            shift_pt_2 = pivot_path[reconnection_end_idx] + cur_dif - 
                        (intersection_end_pt - safety_dis / cur_dif_norm * cur_dif); 
                        
    for (int i = 0; i < reconnection_start_idx; i++)
        res_path[i] = pivot_path[i] + cur_dif;
    int step_num = collision_start_idx - reconnection_start_idx;
    for (int i = reconnection_start_idx; i < collision_start_idx; i++) {
        float shift_ratio = (float)(i - reconnection_start_idx) / (float)step_num;
        res_path[i] = pivot_path[reconnection_start_idx] + cur_dif + shift_ratio * shift_pt_1;
    }
    for (int i = collision_start_idx; i < collision_end_idx; i++) {
        Point2f intersection_pt = GetClosestIntersectionPt(obstacle, pivot_path[i], pivot_path[i] + cur_dif, pivot_path[i]);
        res_path[i] = intersection_pt - safety_dis / cur_dif_norm * cur_dif;
    }
    step_num = reconnection_end_idx - collision_end_idx;
    for (int i = collision_end_idx; i < reconnection_end_idx; i++) {
        float shift_ratio = (float) (i - collision_end_idx) / step_num;
        res_path[i] = intersection_end_pt - safety_dis / cur_dif_norm * cur_dif + shift_ratio * shift_pt_2;
    }
    for (int i = reconnection_end_idx; i < pivot_path.size(); i++)
        res_path[i] = pivot_path[i] + cur_dif;

    // postprocessing
    int truncation_idx = SearchByDistance(res_path, cur_target_pt);
    res_path.erase(res_path.begin() + truncation_idx, res_path.end());
    Point2f temp_end = res_path.back(), segment = cur_target_pt - temp_end;
    float segment_length = cv::norm(segment), temp_path_length = cv::norm(temp_end - res_path.front()), 
            interpolation_ratio;
    int segment_pt_num;
    if (temp_path_length > 1)  // in case temp_path_length becomes 0
        segment_pt_num = segment_length / temp_path_length * res_path.size();
    else
        segment_pt_num = 100;
    for (int interpolation_idx = 1; interpolation_idx <= segment_pt_num; interpolation_idx++) {
        interpolation_ratio = (float)interpolation_idx / segment_pt_num;
        res_path.push_back(temp_end + segment * interpolation_ratio);
    }
    // std::cout << "OK5\n";
    return res_path; 
}

vector<float>  GetLocalPathWidth2D( const vector<Point2f>& path, 
                                    vector<PolygonObstacle>& obstacles,
                                    Size2f config_size = Size2f(640, 480)) {
    vector<float> local_path_width(path.size(), 0);
    vector<int> path_width_type(path.size(), 0);  // 0: two-side free, 1: one-side free, one-side obstacle
                                                  // 2: two-side obstacle
    float slope, x_max = config_size.width, y_max = config_size.height,
          cur_x, cur_y,
          x_intercept, y_intercept,
          x_at_y_max, y_at_x_max;

    for (int i = 1; i < path.size() - 1; i++) {
        cur_x = path[i].x;
        cur_y = path[i].y;
        slope = -(path[i + 1].x - path[i - 1].x) / (path[i + 1].y - path[i - 1].y);
        vector<Point2f> boundary_insection_pts;

        y_intercept = cur_y + slope * (0 - cur_x);
        if (0 <= y_intercept && y_intercept <= y_max)
            boundary_insection_pts.push_back(Point2f(0, y_intercept));
        x_intercept = cur_x - cur_y / slope;
        if (0 <= x_intercept && x_intercept <= x_max)
            boundary_insection_pts.push_back(Point2f(x_intercept, 0));
        x_at_y_max = cur_x + (y_max - cur_y) / slope;
        if (0 <= x_at_y_max && x_at_y_max <= x_max)
            boundary_insection_pts.push_back(Point2f(x_at_y_max, y_max));
        y_at_x_max = cur_x + slope * (x_max - cur_x);
        if (0 <= y_at_x_max && y_at_x_max <= y_max)
            boundary_insection_pts.push_back(Point2f(x_max, y_at_x_max));
        
        // std::cout << "OK_1\n";
        if (boundary_insection_pts.empty()) {
            std::cout << "Error!\n";
            std::cout << path[i] << '\n' << "slope: " << slope << '\n';
        }
        Point2f end_pt1 = boundary_insection_pts[0], end_pt2 = boundary_insection_pts[1],
                direction1 = end_pt1 - path[i], direiction2 = end_pt2 - path[i];
        vector<Point2f> obs_intersection_pts, direction1_pts, direction2_pts;
        for (PolygonObstacle& cur_obs : obstacles) {
            if (ObstacleFree(cur_obs, end_pt1, end_pt2))
                continue;
            Point2f cur_obs_intersection_pt = GetClosestIntersectionPt(cur_obs, end_pt1, end_pt2, path[i]);
            obs_intersection_pts.push_back(cur_obs_intersection_pt);
        }
        
        if (obs_intersection_pts.empty())
            local_path_width[i] = cv::norm(end_pt1 - end_pt2);
        else {
            for (Point2f cur_pt : obs_intersection_pts) {
                Point2f cur_vec = cur_pt - path[i];
                if (cur_vec.x * direction1.x + cur_vec.y * direction1.y > 0)
                    direction1_pts.push_back(cur_pt);
                else
                    direction2_pts.push_back(cur_pt);
            }
            if (!direction1_pts.empty()) {
                path_width_type[i]++;
                for (Point2f& cur_direction1_pt : direction1_pts) {
                    if (normSqr(cur_direction1_pt - path[i]) < normSqr(end_pt1 - path[i]))
                        end_pt1 = cur_direction1_pt;
                }
            }
            if (!direction2_pts.empty()) {
                path_width_type[i]++;
                for (Point2f& cur_direction2_pt : direction2_pts)
                    if (normSqr(cur_direction2_pt - path[i]) < normSqr(end_pt2 - path[i]))
                        end_pt2 = cur_direction2_pt;
            }
            local_path_width[i] = cv::norm(end_pt1 - end_pt2);
        }
    }

    if (local_path_width.size() >= 2) {
        local_path_width.front() = local_path_width[1];
        local_path_width.back() = local_path_width[local_path_width.size() - 2];
    }
    return local_path_width;
}

vector<vector<int>> FilterRawPassagePairsByVisibility(const vector<PolygonObstacle>& obstacles, 
                                                    const vector<vector<int>>& raw_passage_pairs) {
    vector<vector<int>> res;
    vector<set<int>> valid_passage_pair_set(obstacles.size());
    for (const vector<int>& cur_passage_pair : raw_passage_pairs)
        valid_passage_pair_set[cur_passage_pair[0]].insert(cur_passage_pair[1]);

    for (int i = 0; i < valid_passage_pair_set.size(); i++) {
        for (auto it = valid_passage_pair_set[i].begin(); it != valid_passage_pair_set[i].end();) {
            if (ObstacleFreeVecForPassage(obstacles, i, *it) == false)
                it = valid_passage_pair_set[i].erase(it);  // Be careful when dynamically deleting a set element
            else
                it++;
        }
    }

    for (int i = 0; i < raw_passage_pairs.size(); i++) {
        if (valid_passage_pair_set[raw_passage_pairs[i][0]].find(raw_passage_pairs[i][1]) != valid_passage_pair_set[raw_passage_pairs[i][0]].end())
            res.push_back(raw_passage_pairs[i]);
    }
    return res;
}

vector<vector<int>> FilterPassagePairsBySortedWidth(const vector<PolygonObstacle>& obstacles,
                                                    const vector<vector<int>>& raw_passage_pairs) {
    vector<vector<int>> res;
    vector<vector<int>> passage_pairs_mutual_map(obstacles.size());
    for (const vector<int>& cur_passage_pair : raw_passage_pairs) {
        passage_pairs_mutual_map[cur_passage_pair[0]].push_back(cur_passage_pair[1]);
        passage_pairs_mutual_map[cur_passage_pair[1]].push_back(cur_passage_pair[0]);
    }

    vector<int> obs_idx_2_candidate_vec(obstacles.size(), -1);
    for (int i = 0; i < obstacles.size(); i++) {
        if (passage_pairs_mutual_map[i].size() == 0)
            continue;
        vector<pair<float, int>> width_to_obs_idx;
        PolygonObstacle obs1 = obstacles[i];
        for (int j = 0; j < passage_pairs_mutual_map[i].size(); j++) {
            int obs_idx_2 = passage_pairs_mutual_map[i][j];
            PolygonObstacle obs2 = obstacles[obs_idx_2];
            vector<Point2f> cur_passage_inner_ends = GetPassageInnerEnds(obs1, obs2);
            float cur_passage_width = cv::norm(cur_passage_inner_ends[0] - cur_passage_inner_ends[1]);
            width_to_obs_idx.push_back(make_pair(cur_passage_width, obs_idx_2));
        }
        std::sort(width_to_obs_idx.begin(), width_to_obs_idx.end());
        obs_idx_2_candidate_vec[i] = width_to_obs_idx[0].second;
    }

    for (int i = 0; i < obstacles.size(); i++) {
        if (passage_pairs_mutual_map[i].size() == 0)
            continue;
        int obs_idx_2 = obs_idx_2_candidate_vec[i];
        if (obs_idx_2 > i && i == obs_idx_2_candidate_vec[obs_idx_2])
            res.push_back({i, obs_idx_2});
    }
    return res;    
}

vector<vector<int>> GetPassagesPathPasses(const vector<PolygonObstacle>& obstacles, 
                                        const vector<RRTStarNode*>& path) {
    // Raw, non-smoothed path is preferred for efficiency.
    vector<vector<int>> raw_passage_pairs;
    vector<Point2f> obs_centroids = GetObstaclesCentroids(obstacles);
    
    for (int i = 0; i < path.size() - 1; i++) 
        for (int j = 0; j < obs_centroids.size() - 1; j++) 
            for (int k = j + 1; k < obs_centroids.size(); k++) 
                if (SegmentIntersection(path[i]->pos, path[i + 1]->pos, obs_centroids[j], obs_centroids[k]) == true) 
                    raw_passage_pairs.push_back({j, k});
    // Filter invalid passages
    vector<vector<int>> physical_valid_passage_pairs = FilterRawPassagePairsByVisibility(obstacles, raw_passage_pairs);
    return FilterPassagePairsBySortedWidth(obstacles, physical_valid_passage_pairs);
}

vector<vector<Point2f>> GetPassageIntersectionsOfPathSet(const vector<PolygonObstacle>& obstacles, 
                                                        const vector<vector<int>>& passage_pairs,
                                                        const vector<Point2f>& smooth_pivot_path, 
                                                        vector<vector<int>>& path_node_intersection_idx_log,
                                                        const vector<Point2f>& initial_feedback_pts, 
                                                        int pivot_idx) {
    int pts_num = initial_feedback_pts.size();
    vector<vector<Point2f>> res(pts_num, vector<Point2f>(passage_pairs.size()));
    path_node_intersection_idx_log = vector<vector<int>>(pts_num, vector<int>(passage_pairs.size(), 0));
    vector<Point2f> obs_centroids = GetObstaclesCentroids(obstacles);

    for (int i = 0; i < pts_num; i++) {
        int path_node_idx = 0;
        Point2f pt_dif = initial_feedback_pts[i] - initial_feedback_pts[pivot_idx];
        for (int j = 0; j < passage_pairs.size(); j++) {
            // cout << "Testing passage: " << passage_pairs[j][0] << "---" << passage_pairs[j][1] << "\n";
            Point2f obs_centroid_1 = obs_centroids[passage_pairs[j][0]],
                    obs_centroid_2 = obs_centroids[passage_pairs[j][1]],
                    direction_obs_1_2 = (obs_centroid_2 - obs_centroid_1) / cv::norm(obs_centroid_2 - obs_centroid_1),
                    extended_obs_centroid_1 = obs_centroid_1 - 300 * direction_obs_1_2,
                    extended_obs_centroid_2 = obs_centroid_2 + 300 * direction_obs_1_2;
            // cout << "Obstacle centroid 1: " << obs_centroid_1 << " Obstacle centroid 2: " << obs_centroid_2 << "\n";
            // cout << "Extended obstacle centroid 1: " << extended_obs_centroid_1 << " Extended obstacle centroid 2: " << extended_obs_centroid_2 << "\n";
            for (int k = path_node_idx; k < smooth_pivot_path.size() - 1; k++) {
                if (SegmentIntersection(extended_obs_centroid_1, extended_obs_centroid_2, smooth_pivot_path[k] + pt_dif, smooth_pivot_path[k + 1] + pt_dif)) {
                    res[i][j] = GetSegmentsIntersectionPt(extended_obs_centroid_1, extended_obs_centroid_2, smooth_pivot_path[k] + pt_dif, smooth_pivot_path[k + 1] + pt_dif);
                    // cout << "Intersection detected with point:\n" << res[i][j] << '\n';
                    path_node_intersection_idx_log[i][j] = k;
                    // In theory, the same path segment can pass two or more passages, so do not use path_node_idx = k + 1.  
                    // Also, next intersection can be placed before the current one, simply setting path_node = 0 is ok. Here, locality search is used.
                    path_node_idx = (k - (int)smooth_pivot_path.size() / 5) < 0 ? 0 : k - (int)smooth_pivot_path.size() / 5; 
                    break;
                }
            }
        }
    }
    cout << "Intersection pts of path set:\n";
    for (auto pt_vec : res) {
        for (auto pt : pt_vec) 
            cout << pt << ", ";
        cout << "\n";
    }
    return res;
}

vector<Point2f> GetPivotPathRepositionPts(const vector<PolygonObstacle>& obstacles, 
                                        const vector<vector<int>>& passage_pairs, 
                                        const vector<vector<Point2f>>& intersection_points, 
                                        int pivot_idx) {
    vector<Point2f> chord_ends, passage_inner_ends, res(passage_pairs.size());
    vector<Point2f> obs_centroids = GetObstaclesCentroids(obstacles);

    int pts_num = intersection_points.size();
    for (int i = 0; i < passage_pairs.size(); i++) {
        vector<Point2f> cur_passage_intersection_pts(pts_num);
        for (int j = 0; j < pts_num; j++)
            cur_passage_intersection_pts[j] = intersection_points[j][i];
        chord_ends = GetEndsOfColinearPts(cur_passage_intersection_pts);
        passage_inner_ends = GetPassageInnerEnds(obstacles[passage_pairs[i][0]], obstacles[passage_pairs[i][1]]);
/*         cout << "Passage inner ends:\n"
             << passage_inner_ends[0] << "-" << passage_inner_ends[1] << '\n';
        cout << "Chord ends:\n"
             << chord_ends[0] << "-" << chord_ends[1] << '\n';
        cout << "Passage intersection point:\n"
             << cur_passage_intersection_pts[pivot_idx] << '\n'; */

        Point2f reposition_intersection_pt;
        if (normSqr(chord_ends[1] - chord_ends[0]) < normSqr(passage_inner_ends[1] - passage_inner_ends[0])
            && OnSegment(passage_inner_ends[0], passage_inner_ends[1], chord_ends[0])
            && OnSegment(passage_inner_ends[0], passage_inner_ends[1], chord_ends[1])) {
                // cout << "case 1\n";
                res[i] = cur_passage_intersection_pts[pivot_idx];
                continue;
            }
        else if (normSqr(chord_ends[1] - chord_ends[0]) < normSqr(passage_inner_ends[1] - passage_inner_ends[0])) {
            if (OnSegment(passage_inner_ends[0], passage_inner_ends[1], chord_ends[0])) {
                if (normSqr(passage_inner_ends[0] - chord_ends[1]) <= normSqr(passage_inner_ends[1] - chord_ends[1]))
                    reposition_intersection_pt = cur_passage_intersection_pts[pivot_idx] + 1.2 * (passage_inner_ends[0] - chord_ends[1]);
                else 
                    reposition_intersection_pt = cur_passage_intersection_pts[pivot_idx] + 1.2 * (passage_inner_ends[1] - chord_ends[1]);
            }
            else {
                if (normSqr(passage_inner_ends[0] - chord_ends[0]) <= normSqr(passage_inner_ends[1] - chord_ends[0]))
                    reposition_intersection_pt = cur_passage_intersection_pts[pivot_idx] + 1.2 * (passage_inner_ends[0] - chord_ends[0]);
                else 
                    reposition_intersection_pt = cur_passage_intersection_pts[pivot_idx] + 1.2 * (passage_inner_ends[1] - chord_ends[0]);                
            }
            // cout << "Reposition intersection point:\n" 
            //      << reposition_intersection_pt << '\n';
            res[i] = reposition_intersection_pt;
            continue;
        }
        
        Point2f chord_center = (chord_ends[0] + chord_ends[1]) / 2,
                passage_inner_center = (passage_inner_ends[0] + passage_inner_ends[1]) / 2,
                chord_to_passage_center_dif = passage_inner_center - chord_center,
                pivot_path_intersection_pt = cur_passage_intersection_pts[pivot_idx];
        reposition_intersection_pt = pivot_path_intersection_pt + chord_to_passage_center_dif;
        if (OnSegment(passage_inner_ends[0], passage_inner_ends[1], reposition_intersection_pt) == false) {
            if (normSqr(reposition_intersection_pt - passage_inner_ends[0]) <= normSqr(reposition_intersection_pt - passage_inner_ends[1]))
                reposition_intersection_pt = passage_inner_ends[0] + 0.2 * (passage_inner_ends[1] - passage_inner_ends[0]);
            else 
                reposition_intersection_pt = passage_inner_ends[1] + 0.2 * (passage_inner_ends[0] - passage_inner_ends[1]);
        }
        // cout << "Reposition intersection point:\n" 
        //         << reposition_intersection_pt << '\n';        
        res[i] = reposition_intersection_pt;
    }
    return res;
}

vector<vector<Point2f>> AdjustRepositionPtsForPathSet(const vector<PolygonObstacle>& obstacles, 
                                                    const vector<vector<int>>& passage_pairs,
                                                    const vector<vector<Point2f>>& path_set_intersection_pts,
                                                    int pivot_idx) {
    vector<vector<Point2f>> res = path_set_intersection_pts;
    int pts_num = res.size();
    vector<Point2f> chord_ends, passage_inner_ends;
    for (int i = 0; i < passage_pairs.size(); i++) {
        vector<Point2f> cur_passage_intersection_pts(pts_num);
        for (int j = 0; j < pts_num; j++)
            cur_passage_intersection_pts[j] = res[j][i];
        chord_ends = GetEndsOfColinearPts(cur_passage_intersection_pts);
        passage_inner_ends = GetPassageInnerEnds(obstacles[passage_pairs[i][0]], obstacles[passage_pairs[i][1]]); 
        
        if (OnSegment(passage_inner_ends[0], passage_inner_ends[1], chord_ends[0])
            && OnSegment(passage_inner_ends[0], passage_inner_ends[1], chord_ends[1])) {
                continue;
            }
        
        Point2f passage_direction_1 = passage_inner_ends[0] - res[pivot_idx][i],
                passage_direction_2 = passage_inner_ends[1] - res[pivot_idx][i];
        float safety_interior_distance = 15,
              interior_distance_1 = cv::norm(passage_direction_1),
              interior_distance_2 = cv::norm(passage_direction_2),
              max_dif_length_1 = 1e-5,
              max_dif_length_2 = 1e-5;
        if (interior_distance_1 > safety_interior_distance)
            interior_distance_1 -= safety_interior_distance;
        if (interior_distance_2 > safety_interior_distance)
            interior_distance_2 -= safety_interior_distance;
            
        for (int j = 0; j < pts_num; j++) {
            if (j == pivot_idx)
                continue;
            Point2f intersection_pt_dif = res[j][i] - res[pivot_idx][i];
            if (passage_direction_1.dot(intersection_pt_dif) > 0) {
                max_dif_length_1 = max(max_dif_length_1, (float)cv::norm(intersection_pt_dif));
            }
            else {
                max_dif_length_2 = max(max_dif_length_2, (float)cv::norm(intersection_pt_dif));
            }
        }
        float compression_ritio = min(interior_distance_1 / max_dif_length_1, interior_distance_2 / max_dif_length_2);
        for (int j = 0; j < pts_num; j++) {
            if (j == pivot_idx)
                continue;
            // cout << "Original intersection pt: " << res[j][i] << " ";
            res[j][i] = res[pivot_idx][i] + (res[j][i] - res[pivot_idx][i]) * compression_ritio;
            // cout << "Repositioned intersction pt: " << res[j][i] << '\n';
        }
    }
    return res;
}

void DeformPath(vector<Point2f>& path, 
                const vector<Point2f>& intersection_pts, 
                const vector<Point2f>& reposition_intersection_pts, 
                vector<int> path_intersection_idx) {
    vector<float> accumulated_path_length(path.size(), 0);
    for (int i = 1; i < accumulated_path_length.size(); i++) {
        accumulated_path_length[i] = accumulated_path_length[i - 1] + cv::norm(path[i] - path[i - 1]);
    }
        
    vector<int> reposition_idx(path_intersection_idx.size() + 2, 0);
    reposition_idx.back() = path.size() - 1;
    for (int i = 0; i < path_intersection_idx.size(); i++)
        reposition_idx[i + 1] = path_intersection_idx[i];

    float total_path_length = accumulated_path_length.back();
    Point2f cur_shift, next_shift, pre_shift;
    for (int i = 0; i < reposition_idx.size() - 1; i++) {
        int cur_path_idx = reposition_idx[i], next_path_idx = reposition_idx[i + 1];
        float cur_path_length_parameter = accumulated_path_length[cur_path_idx] / total_path_length,
              next_path_length_parameter = accumulated_path_length[next_path_idx] / total_path_length,
              path_segment_length_parameter = next_path_length_parameter - cur_path_length_parameter;
        if (cur_path_idx == 0) 
            cur_shift = Point2f(0, 0);
        else
            cur_shift = reposition_intersection_pts[i - 1] - intersection_pts[i - 1];
        if (next_path_idx == path.size() - 1) 
            next_shift = Point2f(0, 0);
        else
            next_shift = reposition_intersection_pts[i] - intersection_pts[i];
        
        for (int j = cur_path_idx; j < next_path_idx; j++) {
            float path_length_parameter = accumulated_path_length[j] / total_path_length,
                  ratio = (path_length_parameter - cur_path_length_parameter) / path_segment_length_parameter;
            path[j] = path[j] + ratio * next_shift + (1 - ratio) * cur_shift;
            // cout << path_length_parameter << ", " << ratio << ", " << next_shift << ", " << cur_shift << '\n';
        }
    }
}

void DeformPathWithTargetPt(vector<Point2f>& path, 
                            const vector<Point2f>& intersection_pts, 
                            const vector<Point2f>& reposition_intersection_pts, 
                            vector<int> path_intersection_idx) {
    vector<float> accumulated_path_length(path.size(), 0);
    for (int i = 1; i < accumulated_path_length.size(); i++) {
        accumulated_path_length[i] = accumulated_path_length[i - 1] + cv::norm(path[i] - path[i - 1]);
    }
        
    vector<int> reposition_idx(path_intersection_idx.size() + 2, 0);
    reposition_idx.back() = path.size() - 1;
    for (int i = 0; i < path_intersection_idx.size(); i++)
        reposition_idx[i + 1] = path_intersection_idx[i];

    float total_path_length = accumulated_path_length.back();
    Point2f cur_shift, next_shift, pre_shift;
    for (int i = 0; i < reposition_idx.size() - 1; i++) {
        int cur_path_idx = reposition_idx[i], next_path_idx = reposition_idx[i + 1];
        float cur_path_length_parameter = accumulated_path_length[cur_path_idx] / total_path_length,
              next_path_length_parameter = accumulated_path_length[next_path_idx] / total_path_length,
              path_segment_length_parameter = next_path_length_parameter - cur_path_length_parameter;
        if (cur_path_idx == 0) 
            cur_shift = Point2f(0, 0);
        else
            cur_shift = reposition_intersection_pts[i - 1] - intersection_pts[i - 1];
        // if (next_path_idx == path.size() - 1) 
        //   next_shift = Point2f(0, 0);
        // else
        next_shift = reposition_intersection_pts[i] - intersection_pts[i];
        
        for (int j = cur_path_idx; j < next_path_idx; j++) {
            float path_length_parameter = accumulated_path_length[j] / total_path_length,
                  ratio = (path_length_parameter - cur_path_length_parameter) / path_segment_length_parameter;
            path[j] = path[j] + ratio * next_shift + (1 - ratio) * cur_shift;
            // cout << path_length_parameter << ", " << ratio << ", " << next_shift << ", " << cur_shift << '\n';
        }
    }
    path.back() = reposition_intersection_pts.back();
}

vector<vector<Point2f>> GeneratePathSetInGeneralCondition(const vector<Point2f>& initial_feedback_pts, 
                                                          const vector<Point2f>& target_feedback_pts, 
                                                          int pivot_idx,
                                                          float feedback_pts_radius,
                                                          const vector<PolygonObstacle>& obs,
                                                          Mat source_img) {
    // vector<vector<Point2f>> res_path_set(initial_feedback_pts.size());   

    RRTStarPlanner planner(initial_feedback_pts[pivot_idx], target_feedback_pts[pivot_idx], obs, 25);
    bool plan_success = planner.Plan(source_img, feedback_pts_radius, true);
    if (!plan_success) {
        cout << "Path planning failed! An empty path set is reutrned.\n";
        return {};
    }

    vector<RRTStarNode*> pivot_path = planner.GetPath();
    vector<RRTStarNode*> sparse_pivot_path;
    for (int i = 0; i < pivot_path.size(); i += 1)
        sparse_pivot_path.push_back(pivot_path[i]);
    if (sparse_pivot_path.back() != pivot_path.back())
        sparse_pivot_path.push_back(pivot_path.back());
    vector<Point2f> smooth_pivot_path = QuadraticBSplineSmoothing(sparse_pivot_path);

    vector<vector<int>> passage_passed = GetPassagesPathPasses(obs, pivot_path);
    vector<vector<int>> intersection_idx; 
    vector<vector<Point2f>> passage_intersection_pts = GetPassageIntersectionsOfPathSet(obs, passage_passed, smooth_pivot_path, intersection_idx, initial_feedback_pts, pivot_idx);
    vector<Point2f> reposition_points = GetPivotPathRepositionPts(obs, passage_passed, passage_intersection_pts, pivot_idx);
    DeformPath(smooth_pivot_path, passage_intersection_pts[pivot_idx], reposition_points, intersection_idx[pivot_idx]); 

    passage_intersection_pts = GetPassageIntersectionsOfPathSet(obs, passage_passed, smooth_pivot_path, intersection_idx, initial_feedback_pts, pivot_idx);
    vector<vector<Point2f>> adjusted_passage_intersection_pts = AdjustRepositionPtsForPathSet(obs, passage_passed, passage_intersection_pts, pivot_idx);

    vector<vector<Point2f>> res_path_set(initial_feedback_pts.size(), smooth_pivot_path);
    for (int i = 0; i < initial_feedback_pts.size(); i++) {
        if (i == pivot_idx) 
            continue;
        for (Point2f& path_node : res_path_set[i])
            path_node += initial_feedback_pts[i] - initial_feedback_pts[pivot_idx];

        passage_intersection_pts[i].push_back(res_path_set[i].back());
        adjusted_passage_intersection_pts[i].push_back(target_feedback_pts[i]);
        DeformPathWithTargetPt(res_path_set[i], passage_intersection_pts[i], adjusted_passage_intersection_pts[i], intersection_idx[i]);
        //res_path_set[i].back() = target_feedback_pts[i];
    }    
    return res_path_set; 
}

/* Upgraded */
vector<int> RetrievePassedPassages(const vector<RRTStarNode*>& raw_pivot_path, const vector<vector<Point2f>>& passage_pts) {
    vector<int> res;
    for (int path_node_idx = 0; path_node_idx < raw_pivot_path.size() - 1; path_node_idx++) {
        for (int passage_idx = 0; passage_idx < passage_pts.size(); passage_idx++) {
            if (SegmentIntersection(raw_pivot_path[path_node_idx]->pos, raw_pivot_path[path_node_idx + 1]->pos, 
                                    passage_pts[passage_idx][0], passage_pts[passage_idx][1]))
            res.push_back(passage_idx);
        }
    }
    return res;
}

#endif