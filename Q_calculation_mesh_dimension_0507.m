clear
close all
clc

%% Input
% clockwise
DO_vertices = [0, 0, 100, 100;
               0, 50, 50, 0];
vertices_num = size(DO_vertices, 2);
% s_0 = sum(DO_vertices, 2) / vertices_num;
s_0 = [12.5 * 4; 50 - 4 * 6.25];
%s_0 = [50; 25];
e_i_direction = [10; -5];
e_i_direction = e_i_direction / norm(e_i_direction);
e_i_length = 25;
s_d = s_0 + e_i_length * e_i_direction;
 
%% Discretize the DO contour
left_bottom_vertex = DO_vertices(:, 1);
left_up_vertex = DO_vertices(:, 2);
rigth_up_vertex = DO_vertices(:, 3);
right_bottom_vertex = DO_vertices(:, 4);
bottom_length = 100;
side_length = 50;
[DO_mesh_x, DO_mesh_y] = meshgrid(2.5 : 5 : 100, 2.5 : 5 : 50);
DO_mesh = zeros(size(DO_mesh_x));

q_max_value = 0;
e_i = s_d - s_0;
e_i_norm = norm(e_i);
r = 5;
delta_r = 2;
k = 0.02;
gamma = 0.5;
delta_p = 0.1;
delta_d = 0.2;
alpha_0 = pi / 6;
alpha_1 = pi / 6 * 5;
for row = 1 : size(DO_mesh, 1)
    for col = 1 : size(DO_mesh, 2)
        s_g = [DO_mesh_x(row, col); DO_mesh_y(row, col)];
        d_i = s_0 - s_g;
        d_i_norm = norm(d_i);
        alpha_i = acos(e_i' * d_i / (e_i_norm * d_i_norm));
        xi = 1 + delta_d;
%         if alpha_i <= alpha_0 || alpha_i >= alpha_1
%             xi = 1 + delta;
%         end
        if alpha_i <= alpha_0 || alpha_i >= alpha_1
            xi = 1 + delta_p;
        end
        s_d_g = s_g + xi * e_i;
        
        % integrate
        step_num = 100;
        step_len = e_i_norm / step_num;
        q_i = 0;
        for i = 1 : step_num
            s_g_i = s_g + i / step_num * xi * e_i;
            s_0_i = s_0 + i / step_num * e_i;
            d_i = s_0_i - s_g_i;
            d_i_norm = norm(d_i);          
            cos_val = e_i' * d_i / (e_i_norm * d_i_norm);
            if cos_val > 1
                cos_val = 1;
            end
            if cos_val < -1
                cos_val = -1;
            end
            alpha_i = acos(cos_val);
          
            lambda_d = 1;
            if d_i_norm <= r
                lambda_d = 1 / (1 - gamma * sin(alpha_i));
            end
            if d_i_norm > r && d_i_norm <= r + delta_r
                lambda_d = 1 / (1 - gamma * sin(alpha_i)) * (1 - (d_i_norm - r) / delta_r * gamma * sin(alpha_i));
            end
            M = exp(-k * d_i_norm) * (1 - gamma * sin(alpha_i)) * lambda_d;
            q_i = q_i + M * step_len;
        end
        DO_mesh(row, col) = q_i;
        if q_i > q_max_value
            q_max_value = q_i;
        end
    end
end
DO_mesh = DO_mesh / q_max_value;
DO_mesh = flipud(DO_mesh);

%% Normalize boundary values
boundary_max = 0;
scale_coeff = 1;
for col = 1 : size(DO_mesh, 2)
    DO_mesh(1, col) = DO_mesh(1, col) * scale_coeff;
    boundary_max = max(DO_mesh(1, col), boundary_max);
    DO_mesh(end, col) = DO_mesh(end, col) * scale_coeff;
    boundary_max = max(DO_mesh(end, col), boundary_max);
end
for row = 2 : size(DO_mesh, 1) - 1
    DO_mesh(row, 1) = DO_mesh(row, 1) * scale_coeff;
    boundary_max = max(DO_mesh(row, 1), boundary_max);
    DO_mesh(row, end) = DO_mesh(row, end) * scale_coeff;
    boundary_max = max(DO_mesh(row, end), boundary_max);
end
DO_mesh(1, :) = DO_mesh(1, :) / boundary_max;
DO_mesh(end, :) = DO_mesh(end, :) / boundary_max;
DO_mesh(2 : end - 1, 1) = DO_mesh(2 : end - 1, 1) / boundary_max;
DO_mesh(2 : end - 1, end) = DO_mesh(2 : end - 1, end) / boundary_max;
%DO_mesh = DO_mesh / boundary_max;

%% figure
fig = figure('Position', [200, 200, 500, 250]);
ax = axes('Parent', fig);
hold(ax, 'off')
ht1 = heatmap(DO_mesh, 'Colormap', jet);
ht1.CellLabelColor = 'none';
ht1.CellLabelFormat = '%0.2f';
ht1.ColorbarVisible = 'off';
set(gcf, 'Renderer', 'Painters');
%print(fig, './Figure/Rectangle_Centered_Mesh_Diag_Normalized_3', '-depsc')
% print(fig, 'Rectangle_Centered_Mesh_0_PNG', '-dpng')
%exportgraphics(fig, 'Rectangle_Centered_Mesh_0.png','Resolution', 2000)  % Less margin in the
%outputed image