clear
close all
clc

%% Input
% clockwise
DO_vertices = [0, 0, 100, 100;
               0, 50, 50, 0];
vertices_num = size(DO_vertices, 2);
% s_0 = sum(DO_vertices, 2) / vertices_num;
s_0 = [12.5 * 2; 50 - 2 * 6.25];
% s_0 = [50; 25];
e_i_direction = [2; -1];
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

M_max_value = 0;
e_i = s_d - s_0;
e_i_norm = norm(e_i);
r = 5;
delta_r = 2;
k_d = 0.01;
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
        %alpha_i = acos(e_i' * d_i / (e_i_norm * d_i_norm));
        cos_val = e_i' * d_i / (norm(e_i) * norm(d_i));
        if cos_val < -1
            cos_val = -1;
        elseif cos_val > 1
            cos_val = 1;
        end
        alpha_i = acos(cos_val);        
           
        h_1 = exp(-k_d * d_i_norm);
        h_2 = 1 - gamma * sin(alpha_i);
        if d_i_norm <= r
            h_3 = 1 / h_2;
        elseif r < d_i_norm && d_i_norm < r + delta_r
            h_3 = (1 - (d_i_norm - r) / delta_r * gamma * sin(alpha_i)) / h_2;
        else
            h_3 = 1;
        end
        
        M_i = h_1 * h_2 * h_3;
        DO_mesh(row, col) = M_i;
        if M_i > M_max_value
            M_max_value = M_i;
        end
    end
end
DO_mesh = DO_mesh / M_max_value;
DO_mesh = flipud(DO_mesh);

%% Normalize boundary values
% boundary_max = 0;
% scale_coeff = 3;
% for col = 1 : size(DO_mesh, 2)
%     DO_mesh(1, col) = DO_mesh(1, col) * scale_coeff;
%     boundary_max = max(DO_mesh(1, col), boundary_max);
%     DO_mesh(end, col) = DO_mesh(end, col) * scale_coeff;
%     boundary_max = max(DO_mesh(end, col), boundary_max);
% end
% for row = 2 : size(DO_mesh, 1) - 1
%     DO_mesh(row, 1) = DO_mesh(row, 1) * scale_coeff;
%     boundary_max = max(DO_mesh(row, 1), boundary_max);
%     DO_mesh(row, end) = DO_mesh(row, end) * scale_coeff;
%     boundary_max = max(DO_mesh(row, end), boundary_max);
% end
% DO_mesh = DO_mesh / boundary_max;

%% figure
fig = figure('Position', [200, 200, 500, 250]);
ax = axes('Parent', fig);
hold(ax, 'off')
ht1 = heatmap(DO_mesh, 'Colormap', jet);
ht1.CellLabelColor = 'none';
ht1.CellLabelFormat = '%0.2f';
ht1.ColorbarVisible = 'off';
set(gcf, 'Renderer', 'Painters');
% print(fig, './Figure/Elementary_Case_1', '-depsc')
% print(fig, './Figure/Elementary_Case_1', '-dpng')
%exportgraphics(fig, 'Rectangle_Centered_Mesh_0.png','Resolution', 2000)  % Less margin in the
%outputed image