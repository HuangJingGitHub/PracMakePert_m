clear
close all
clc

%% Input
% clockwise
DO_vertices = [0, 0, 100, 100;
               0, 50, 50, 0];
vertices_num = size(DO_vertices, 2);
s_1 = [40, 50, 60;
       37.5, 25, 12.5];
ref_direction = [5; 4];
ref_direction = ref_direction / norm(ref_direction);
e_i_length = [10, 20, 10];
s_0 = zeros(2, 3);
for i = 1 : 3
    s_0(:, i) = s_1(:, i) - e_i_length(1, i) * ref_direction;
end
% s_0 = [20, 20; 
%        15, 35];
e_i_direction = [5, 5, 5; 
                 4, 4, 4];
for col = 1 : size(e_i_direction, 2)
    e_i_direction(:, col) = e_i_direction(:, col) / norm(e_i_direction(:, col));
end
s_d = zeros(size(s_0));
for col = 1 : size(s_0, 2)
    s_d(:, col) = s_0(:, col) + e_i_length(1, col) * e_i_direction(:, col);
end
 
%% Discretize the DO contour
left_bottom_vertex = DO_vertices(:, 1);
left_up_vertex = DO_vertices(:, 2);
rigth_up_vertex = DO_vertices(:, 3);
right_bottom_vertex = DO_vertices(:, 4);
bottom_length = 100;
side_length = 50;
[DO_mesh_x, DO_mesh_y] = meshgrid(2.5 : 5 : 100, 2.5 : 5 : 50);


[M_1, M_2, M_3, M_4, M_5] = compute_metrics(DO_mesh_x, DO_mesh_y, s_0, s_d);

M_1 = M_1 / max(M_1, [], 'all');
M_1 = flipud(M_1);
M_2 = M_2 / max(M_2, [], 'all');
M_2 = flipud(M_2);
M_3 = M_3 / max(M_3, [], 'all');
M_3 = flipud(M_3);
M_4 = M_4 / max(M_4, [], 'all');
M_4 = flipud(M_4);
M_5 = M_5 / max(M_5, [], 'all');
M_5 = flipud(M_5);

%% figure
fig1 = figure('Position', [200, 200, 450, 225]);
ax = axes('Parent', fig1);
hold(ax, 'off')
ht1 = heatmap(M_1, 'Colormap', jet);
ht1.CellLabelColor = 'none';
ht1.CellLabelFormat = '%0.2f';
ht1.ColorbarVisible = 'off';
set(gca, 'Position', [0.0500 0.1165 0.94 0.81])
set(gcf, 'Renderer', 'Painters');
print(fig1, './Figure/Fold_Mesh_1', '-depsc')
%print(fig1, 'Rectangle_Centered_Mesh_0_PNG', '-dpng')
%exportgraphics(fig, 'Rectangle_Centered_Mesh_0.png','Resolution', 2000)  % Less margin in the
%outputed image

fig2 = figure('Position', [220, 200, 450, 225]);
ax = axes('Parent', fig2);
hold(ax, 'off')
ht1 = heatmap(M_2, 'Colormap', jet);
ht1.CellLabelColor = 'none';
ht1.CellLabelFormat = '%0.2f';
ht1.ColorbarVisible = 'off';
set(gca, 'Position', [0.0500 0.1165 0.94 0.81])
set(gcf, 'Renderer', 'Painters');
print(fig2, './Figure/Fold_Mesh_2', '-depsc')

fig3 = figure('Position', [240, 200, 450, 225]);
ax = axes('Parent', fig3);
hold(ax, 'off')
ht1 = heatmap(M_3, 'Colormap', jet);
ht1.CellLabelColor = 'none';
ht1.CellLabelFormat = '%0.2f';
ht1.ColorbarVisible = 'off';
set(gca, 'Position', [0.0500 0.1165 0.94 0.81])
set(gcf, 'Renderer', 'Painters');
print(fig3, './Figure/Fold_Mesh_3', '-depsc')

fig4 = figure('Position', [260, 200, 450, 225]);
ax = axes('Parent', fig4);
hold(ax, 'off')
ht1 = heatmap(M_4, 'Colormap', jet);
ht1.CellLabelColor = 'none';
ht1.CellLabelFormat = '%0.2f';
ht1.ColorbarVisible = 'off';
set(gca, 'Position', [0.0500 0.1165 0.94 0.81])
set(gcf, 'Renderer', 'Painters');
print(fig4, './Figure/Fold_Mesh_4', '-depsc')

fig5 = figure('Position', [280, 200, 450, 225]);
ax = axes('Parent', fig5);
hold(ax, 'off')
ht1 = heatmap(M_5, 'Colormap', jet);
ht1.CellLabelColor = 'none';
ht1.CellLabelFormat = '%0.2f';
ht1.ColorbarVisible = 'off';
set(gca, 'Position', [0.0500 0.1165 0.94 0.81])
set(gcf, 'Renderer', 'Painters');
print(fig5, './Figure/Fold_Mesh_5', '-depsc')


%%
function [M_CM, M_DWCM, M_PCM, M_DWPCM, M_TOM] = compute_metrics(DO_mesh_x, DO_mesh_y, s_0, s_d)
    M_CM = zeros(size(DO_mesh_x));
    M_DWCM = zeros(size(DO_mesh_x));
    M_PCM = zeros(size(DO_mesh_x));
    M_DWPCM = zeros(size(DO_mesh_x));
    M_TOM = zeros(size(DO_mesh_x));

    k = size(s_0, 2);
    
    e_i_vec = s_d - s_0;
    e_g = sum(e_i_vec, 2) / k;
    e_i_length = zeros(1, k);
    for i = 1 : k
        e_i_length(1, i) = norm(e_i_vec(:, i));
    end
  
    [~, original_s_idx] = sort(e_i_length, 'descend');
    max_e_i_idx = original_s_idx(1, 1);
    xi = 1.5 * e_i_length(max_e_i_idx) / norm(e_g);
    
    %e_i_weight = e_i_length / max_e_i_length;
    e_i_weight = e_i_length / sum(e_i_length, 2);

    s_0_centroid = sum(s_0, 2) / k;
    s_d_centroid = sum(s_d, 2) / k;

    s_0_weighted_centroid = zeros(2, 1);
    s_d_weighted_centroid = zeros(2, 1);
    for i = 1 : k
        s_0_weighted_centroid = s_0_weighted_centroid + e_i_weight(1, i) * s_0(:, i);
        s_d_weighted_centroid = s_d_weighted_centroid + e_i_weight(1, i) * s_d(:, i);
    end
    
    
    for row = 1 : size(DO_mesh_x, 1)
        for col = 1 : size(DO_mesh_x, 2)
            s_g = [DO_mesh_x(row, col); DO_mesh_y(row, col)];
            s_d_g = s_g + xi * e_g;

            for j = 1 : k
                s_0_i = s_0(:, j);
                M_j = compute_static_metric(s_g, s_0_i, s_d(:, j));
                M_CM(row, col) = M_CM(row, col) + M_j;
                M_DWCM(row, col) = M_DWCM(row, col) + e_i_weight(1, j) * M_j;
                M_TOM(row, col) = M_TOM(row, col) + integrate_metric(s_g, s_d_g, s_0_i, s_d(:, j));
            end
            M_PCM(row, col) = compute_static_metric(s_g, s_0_centroid, s_d_centroid);
            M_DWPCM(row, col) = compute_static_metric(s_g, s_0_weighted_centroid, s_d_weighted_centroid);
        end
    end
end

%%
function M_i = integrate_metric(s_g, s_d_g, s_0, s_d)
    M_i = 0;
    
    e_g = s_d_g - s_g;
    e_i = s_d - s_0;
    steps = 100;
    delta_path = norm(e_i) / steps;
    for i = 1 : steps - 1
        s_g_cur = s_g + i / steps * e_g;
        s_i_cur = s_0 + i / steps * e_i;
        M_i_cur = compute_static_metric(s_g_cur, s_i_cur, s_d);
        M_i = M_i + M_i_cur * delta_path;
    end
end


%%
function M_i = compute_static_metric(s_g, s_0, s_d) 
        k_d = 0.01;
        gamma = 0.4;
        r = 10;
        delta_r = 2;
        
        d_i = s_0 - s_g;
        d_i_norm = norm(d_i);
        e_i = s_d - s_0;
        cos_val = e_i' * d_i / (norm(e_i) * norm(d_i));
        if cos_val < -1
            cos_val = -1;
        elseif cos_val > 1
            cos_val = 1;
        end
        alpha_i = acos(cos_val);

        h_1 = exp(-k_d * d_i_norm);
        h_2 = 1 - gamma * sin(alpha_i);
        h_3 = 1;
        if d_i_norm <= r
            h_3 = 1 / h_2;
        elseif d_i_norm <= r + delta_r
            h_3 = (1 - (d_i_norm - r) / delta_r * gamma * sin(alpha_i)) / h_2;
        end
        M_i = h_1 * h_2 * h_3;    
end