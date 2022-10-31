clear
close all
clc

%%
%% distance based method:dm  metric based method:mm
fabric_2pt_dm = [8.2, 5.4, 10.8, 10.3, 18.1, 7.0, 14.6, 16.6 8.5, 9.7, 4.5, 18.2];
fabric_2pt_mm = [7.9, 6.5, 7.2, 9.7, 13.5, 5.5, 10.8, 16.5, 6.8, 8.9, 4.8, 16.1];
fabric_3pt_dm = [14.1, 19.5, 10.9, 25.8, 13.9, 24.5, 8.4, 8.6];
fabric_3pt_mm = [10.0, 19.1, 20.5, 13.6, 22.5, 7.6, 9.6];
fabric_4pt_dm = [16.3, 11.8, 37.3, 21.3, 33.7, 12.2];
fabric_4pt_mm = [13.6, 13.7, 31.7, 20.7, 32.7, 12.0];

sponge_1pt_dm = [2.9, 2.9, 3.2, 3.3, 2.9];
sponge_1pt_mm = [2.9, 3.7, 3.3, 3.1, 2.7];
sponge_2pt_dm = [11.7, 3.7, 10.4, 16.0, 13.5, 19.1];
sponge_2pt_mm = [10.5, 2.7, 11.5, 10.9, 13.5, 20.5];
sponge_3pt_dm = [9.6, 11.9, 21.4, 16.7];
sponge_3pt_mm = [8.7, 9.3, 21.3, 17.2];

%%
average_err = zeros(6, 2);
average_err(1, 1) = sum(fabric_2pt_dm, 2) / size(fabric_2pt_dm, 2);
average_err(1, 2) = sum(fabric_2pt_mm, 2) / size(fabric_2pt_mm, 2);
average_err(2, 1) = sum(fabric_3pt_dm, 2) / size(fabric_3pt_dm, 2);
average_err(2, 2) = sum(fabric_3pt_mm, 2) / size(fabric_3pt_mm, 2);
average_err(3, 1) = sum(fabric_4pt_dm, 2) / size(fabric_4pt_dm, 2);
average_err(3, 2) = sum(fabric_4pt_mm, 2) / size(fabric_4pt_mm, 2);

average_err(4, 1) = sum(sponge_1pt_dm, 2) / size(sponge_1pt_dm, 2);
average_err(4, 2) = sum(sponge_1pt_mm, 2) / size(sponge_1pt_mm, 2);
average_err(5, 1) = sum(sponge_2pt_dm, 2) / size(sponge_2pt_dm, 2);
average_err(5, 2) = sum(sponge_2pt_mm, 2) / size(sponge_2pt_mm, 2);
average_err(6, 1) = sum(sponge_3pt_dm, 2) / size(sponge_3pt_dm, 2);
average_err(6, 2) = sum(sponge_3pt_mm, 2) / size(sponge_3pt_mm, 2);

%%
fig = figure('Position', [200, 200, 600, 350]);
bar(average_err);
set(gca, 'FontSize', 14)
ylabel('Feedback Error (px)', 'FontName', 'Arial', 'FontSize', 16);
ax = gca;
ax.XGrid = 'off';
ax.YGrid = 'on';
set(gcf, 'Renderer', 'Painters')
print(fig, './Figure/err_bar_plot', '-depsc')