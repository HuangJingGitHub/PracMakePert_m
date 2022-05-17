clear
close all
clc

%% Initialzie 
[xy_x, xy_y] = meshgrid(0.5 : 2 : 100, 0.5 : 2 : 50);
xy_z = 20 * ones(size(xy_x));
[xz_x, xz_z] = meshgrid(0.5 : 2 : 100, 0.5 : 2 : 20);
xz_y = zeros(size(xz_x));
[yz_y, yz_z] = meshgrid(0.5 : 2 : 50, 0.5 : 2 : 20);
yz_x = zeros(size(yz_y));

%% 
k = 1 / 50000;
l = 100;
xy_z_deformed = xy_z;
for row = 1 : size(xy_z, 1)
    for col = 1 : size(xy_z, 2)
        temp_x = xy_x(row, col);
        xy_z_deformed(row, col) = xy_z(row, col) + k * temp_x * temp_x * (3 * l - temp_x);
    end
end
xz_z_deformed = xz_z;
for row = 1 : size(xz_z, 1)
    for col = 1 : size(xz_z, 2)
        temp_x = xz_x(row, col);
        xz_z_deformed(row, col) = xz_z(row, col) + k * temp_x * temp_x * (3 * l - temp_x);
    end
end

%% Find boundaries
yz_boundary = zeros(3, 2 * size(yz_x, 1) + 2 * size(yz_x, 2) - 4);
cnt = 1;
for col = 1 : size(yz_x, 2)
    yz_boundary(:, cnt) = [yz_x(1, col); yz_y(1, col); yz_z(1, col)];
    cnt = cnt + 1;
end
for row = 2 : size(yz_x, 1)
    yz_boundary(:, cnt) = [yz_x(row, end); yz_y(row, end); yz_z(row, end)];
    cnt = cnt + 1;
end
for col = size(yz_x, 2) - 1 : -1 : 1
    yz_boundary(:, cnt) = [yz_x(end, col); yz_y(end, col); yz_z(end, col)];
    cnt = cnt + 1;
end
for row = size(yz_x, 1) - 1 : -1 : 2
    yz_boundary(:, cnt) = [yz_x(row, 1); yz_y(row, 1); yz_z(row, 1)];
    cnt = cnt + 1;
end
yz_boundary_deformed = yz_boundary;

xy_boundary = zeros(3, size(xy_x, 1) + 2 * size(xy_x, 2) - 4);
xy_boundary_deformed = xy_boundary;
cnt = 1;
for col = 2 : size(xy_x, 2)
    xy_boundary(:, cnt) = [xy_x(1, col); xy_y(1, col); xy_z(1, col)];
    xy_boundary_deformed(:, cnt) = [xy_x(1, col); xy_y(1, col); xy_z_deformed(1, col)];
    cnt = cnt + 1;
end
for row = 2 : size(xy_x, 1)
    xy_boundary(:, cnt) = [xy_x(row, end); xy_y(row, end); xy_z(row, end)];
    xy_boundary_deformed(:, cnt)= [xy_x(row, end); xy_y(row, end); xy_z_deformed(row, end)];
    cnt = cnt + 1;
end
for col = size(xy_x, 2) - 1 : - 1 : 2
    xy_boundary(:, cnt) = [xy_x(end, col); xy_y(end, col); xy_z(end, col)];
    xy_boundary_deformed(:, cnt) = [xy_x(end, col); xy_y(end, col); xy_z_deformed(end, col)];
    cnt = cnt + 1;
end
xz_boundary = zeros(3, size(xz_x, 1) + size(xz_x, 2) - 1);
xz_boundary_deformed = xz_boundary;
cnt = 1;
for col = 1 : size(xz_x, 2)
    xz_boundary(:, cnt) = [xz_x(1, col); xz_y(1, col); xz_z(1, col)];
    xz_boundary_deformed(:, cnt) = [xz_x(1, col); xz_y(1, col); xz_z_deformed(1, col)];
    cnt = cnt + 1;
end
for row = 2 : size(xz_x, 1)
    xz_boundary(:, cnt) = [xz_x(row, end); xz_y(row, end); xz_z(row, end)];
    xz_boundary_deformed(:, cnt) = [xz_x(row, end); xz_y(row, end); xz_z_deformed(row, end)];
    cnt = cnt + 1;
end

%% Find pts with maximum displacements
boundary_set = [yz_boundary, xy_boundary, xz_boundary];
boundary_set_deformed = [yz_boundary_deformed, xy_boundary_deformed, xz_boundary_deformed];
boundary_displacement = zeros(1, size(boundary_set, 2));
for i = 1 : size(boundary_set, 2)
    boundary_displacement(1, i) = norm(boundary_set_deformed(:, i) - boundary_set(:, i));
end

[dis_max, dis_max_idx] = max(boundary_displacement);
threshold = 0.05;
dis_max_idx_set = [];
for i = 1 : size(boundary_displacement, 2)
    if abs(boundary_displacement(1, i) - boundary_displacement(1, dis_max_idx)) <= threshold
        dis_max_idx_set = [dis_max_idx_set, i];
    end
end


%% plot
figure
for row = 1 : size(xy_x, 1)
    scatter3(xy_x(row, :), xy_y(row, :), xy_z(row, :), ...
             'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 1);
    hold on;
    scatter3(xy_x(row, :), xy_y(row, :), xy_z_deformed(row, :), ...
             'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.8);
    hold on;    
end
for row = 1 : size(xz_x, 1)
    scatter3(xz_x(row, :), xz_y(row, :), xz_z(row, :), ...
             'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k');
    hold on;
    scatter3(xz_x(row, :), xz_y(row, :), xz_z_deformed(row, :), ...
             'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.8);    
end
for row = 1 : size(yz_x, 1)
    scatter3(yz_x(row, :), yz_y(row, :), yz_z(row, :), ...
             'MarkerFaceColor', 'g', 'MarkerEdgeColor', 'k');
    hold on;
%     scatter3(yz_x(row, :), yz_y(row, :), deformed_yz_z(row, :), ...
%              'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'k', 'MarkerFaceAlpha', 0.8);    
end
axis equal
axis([0, 100, 0, 50, 0, 60])
