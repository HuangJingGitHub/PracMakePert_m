clear
close all
clc

% google scolar search keyword: deformable object manipualtion
% unfiltered_paper_num = [14100, 14900, 15100, 15600, 16100, 16300, 16800];
% google scolar search keyword: deformable object manipualtion AND robot
unfiltered_paper_num = [3590, 4230, 4570, 5180, 6030, 6750, 7980];
% search keyword: "deformable object" 
% filtered_paper_num = [43, 49, 70, 120, 162, 276, 381];
% search keyword: "deformable object" AND "robot"
filtered_paper_num = [42, 47, 66, 114, 161, 269, 368];


%%
X = categorical({'2017','2018','2019','2020', '2021', '2022', '2023'});
% X = reordercats(X, {'F-1pt','F-2pt','F-3pt','F-4pt', 'S-1pt', 'S-2pt', 'S-3pt'});

fig1 = figure('Position', [200, 200, 1100, 420]);
H1 = subplot(1, 2, 1);
bar(X, unfiltered_paper_num, 'FaceColor', [0.8500 0.3250 0.0980], 'BarWidth', 0.5);
set(gca, 'FontSize', 15)
ylabel('Article Number', 'FontName', 'Arial', 'FontSize', 17);
% l1 = legend({'Distance Sum Method'});
% set(l1, 'Position', [0.2469 0.848 0.2181 0.1202])
ax = gca;
ax.XGrid = 'off';
ax.YGrid = 'on';
ylim([0, 8050]);

H2 = subplot(1, 2, 2);
barg = bar(X, filtered_paper_num, 'FaceColor', [0.8500 0.3250 0.0980], 'BarWidth', 0.5);
set(gca, 'FontSize', 15)
ylabel('Article Number', 'FontName', 'Arial', 'FontSize', 17);
ax = gca;
ax.XGrid = 'off';
ax.YGrid = 'on';
%ylim([0, 63]);
set(gcf, 'Renderer', 'Painters')
% print(fig1, './Figure/combined_bar_plot_update', '-depsc')
