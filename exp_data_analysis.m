clear
close all
clc

%%
%% distance based method:dm  metric based method:mm
fabric_1pt_dm = [2.90, 2.71, 2.96, 2.64, 2.55, 2.80, 2.68, 2.94, 2.45, 2.85, 2.97, 2.72];
fabric_1pt_mm = [2.67, 2.92, 2.66, 2.88, 2.39, 2.98, 2.83, 2.97, 2.85, 3.00, 2.89, 2.89];
% fabric_2pt_dm = [8.2, 5.4, 10.8, 10.3, 18.1, 7.0, 14.6, 16.6, 8.5, 9.7, 4.5, 18.2];
% fabric_2pt_mm = [7.9, 6.5, 7.2, 9.7, 13.5, 5.5, 10.8, 16.5, 6.8, 8.9, 4.8, 16.1];
% fabric_3pt_dm = [14.1, 19.5, 10.9, 25.8, 13.9, 24.5, 8.4, 8.6];
% fabric_3pt_mm = [10.0, 19.1, 10.2, 20.5, 13.6, 22.5, 7.6, 9.6];
% fabric_4pt_dm = [16.3, 11.8, 37.3, 21.3, 33.7, 12.2];
% fabric_4pt_mm = [13.6, 13.7, 31.7, 20.7, 32.7, 12.0];
%% Updated data
fabric_2pt_dm = [8.2, 5.4, 17.1, 10.3, 15.7, 7.0, 14.6, 17.0, 8.5, 9.7, 4.5, 17.0];
fabric_2pt_mm = [7.9, 6.5, 6.8, 9.7, 9.1, 5.5, 10.8, 16.3, 6.8, 8.9, 4.8, 15.9];
fabric_3pt_dm = [14.1, 19.5, 9.9, 25.8, 13.9, 26.4, 8.1, 8.6];
fabric_3pt_mm = [10.0, 19.1, 9.7, 20.5, 13.6, 22.1, 8.1, 9.6];
fabric_4pt_dm = [16.3, 13.5, 34.8, 21.3, 35.9, 11.4];
fabric_4pt_mm = [13.6, 12.1, 23.8, 20.7, 35.8, 10.8];

sponge_1pt_dm = [2.89, 2.94, 2.76, 2.96, 2.94];
sponge_1pt_mm = [2.93, 2.88, 2.85, 2.99, 2.73];
sponge_2pt_dm = [11.7, 3.7, 10.4, 16.0, 13.5, 19.1];
sponge_2pt_mm = [10.5, 2.7, 11.5, 10.9, 13.5, 20.5];
sponge_3pt_dm = [9.6, 11.9, 21.4, 16.7];
sponge_3pt_mm = [8.7, 9.3, 21.3, 17.2];

%%
average_err = zeros(7, 2);
average_err(1, 1) = sum(fabric_1pt_dm, 2) / size(fabric_1pt_dm, 2);
average_err(1, 2) = sum(fabric_1pt_mm, 2) / size(fabric_1pt_mm, 2);
average_err(2, 1) = sum(fabric_2pt_dm, 2) / size(fabric_2pt_dm, 2);
average_err(2, 2) = sum(fabric_2pt_mm, 2) / size(fabric_2pt_mm, 2);
average_err(3, 1) = sum(fabric_3pt_dm, 2) / size(fabric_3pt_dm, 2);
average_err(3, 2) = sum(fabric_3pt_mm, 2) / size(fabric_3pt_mm, 2);
average_err(4, 1) = sum(fabric_4pt_dm, 2) / size(fabric_4pt_dm, 2);
average_err(4, 2) = sum(fabric_4pt_mm, 2) / size(fabric_4pt_mm, 2);

average_err(5, 1) = sum(sponge_1pt_dm, 2) / size(sponge_1pt_dm, 2);
average_err(5, 2) = sum(sponge_1pt_mm, 2) / size(sponge_1pt_mm, 2);
average_err(6, 1) = sum(sponge_2pt_dm, 2) / size(sponge_2pt_dm, 2);
average_err(6, 2) = sum(sponge_2pt_mm, 2) / size(sponge_2pt_mm, 2);
average_err(7, 1) = sum(sponge_3pt_dm, 2) / size(sponge_3pt_dm, 2);
average_err(7, 2) = sum(sponge_3pt_mm, 2) / size(sponge_3pt_mm, 2);

%% case by case comparison (cc)
error_reduction_cc = zeros(7, 1);
[err_reduction_f_1pt, error_reduction_cc(1, 1)] = computeReductionRatebyCase(fabric_1pt_dm, fabric_1pt_mm);
[err_reduction_f_2pt, error_reduction_cc(2, 1)] = computeReductionRatebyCase(fabric_2pt_dm, fabric_2pt_mm);
[err_reduction_f_3pt, error_reduction_cc(3, 1)] = computeReductionRatebyCase(fabric_3pt_dm, fabric_3pt_mm);
[err_reduction_f_4pt, error_reduction_cc(4, 1)] = computeReductionRatebyCase(fabric_4pt_dm, fabric_4pt_mm);
[err_reduction_s_1pt, error_reduction_cc(5, 1)] = computeReductionRatebyCase(sponge_1pt_dm, sponge_1pt_mm);
[err_reduction_s_2pt, error_reduction_cc(6, 1)] = computeReductionRatebyCase(sponge_2pt_dm, sponge_2pt_mm);
[err_reduction_s_3pt, error_reduction_cc(7, 1)] = computeReductionRatebyCase(sponge_3pt_dm, sponge_3pt_mm);
%%
err_reduction = zeros(7, 2);
for i = 1 : 7
    err_reduction(i, 1) = (average_err(i, 1) - average_err(i, 2)) / average_err(i, 1);
end
err_reduction(1, 1) = 0.000001;

for i = 1 : size(fabric_1pt_dm, 2)
    err_reduction(1, 2) = max(err_reduction(1, 2), (fabric_1pt_dm(1, i) - fabric_1pt_mm(1, i)) / fabric_1pt_dm(1, i));
end
for i = 1 : size(fabric_2pt_dm, 2)
    err_reduction(2, 2) = max(err_reduction(2, 2), (fabric_2pt_dm(1, i) - fabric_2pt_mm(1, i)) / fabric_2pt_dm(1, i));
end
for i = 1 : size(fabric_3pt_dm, 2)
    err_reduction(3, 2) = max(err_reduction(3, 2), (fabric_3pt_dm(1, i) - fabric_3pt_mm(1, i)) / fabric_3pt_dm(1, i));
end
for i = 1 : size(fabric_4pt_dm, 2)
    err_reduction(4, 2) = max(err_reduction(4, 2), (fabric_4pt_dm(1, i) - fabric_4pt_mm(1, i)) / fabric_4pt_dm(1, i));
end
for i = 1 : size(sponge_1pt_dm, 2)
    err_reduction(5, 2) = max(err_reduction(5, 2), (sponge_1pt_dm(1, i) - sponge_1pt_mm(1, i)) / sponge_1pt_dm(1, i));
end
for i = 1 : size(sponge_2pt_dm, 2)
    err_reduction(6, 2) = max(err_reduction(6, 2), (sponge_2pt_dm(1, i) - sponge_2pt_mm(1, i)) / sponge_2pt_dm(1, i));
end
for i = 1 : size(sponge_3pt_dm, 2)
    err_reduction(7, 2) = max(err_reduction(7, 2), (sponge_3pt_dm(1, i) - sponge_3pt_mm(1, i)) / sponge_3pt_dm(1, i));
end
err_reduction = 100 * err_reduction;

%%
X = categorical({'F-1pt','F-2pt','F-3pt','F-4pt', 'S-1pt', 'S-2pt', 'S-3pt'});
X = reordercats(X,{'F-1pt','F-2pt','F-3pt','F-4pt', 'S-1pt', 'S-2pt', 'S-3pt'});

fig1 = figure('Position', [200, 200, 600, 350]);
bar(X, average_err);
set(gca, 'FontSize', 14)
ylabel('Composite Error Norm (px)', 'FontName', 'Arial', 'FontSize', 16);
l1 = legend({'Distance Sum Metric', 'Task-Oriented Metric'});
set(l1, 'Position', [0.554 0.78 0.3467 0.1329])
ax = gca;
ax.XGrid = 'off';
ax.YGrid = 'on';
% set(gcf, 'Renderer', 'Painters')
% print(fig1, './Figure/err_bar_plot', '-depsc')

fig2 = figure('Position', [200, 200, 600, 350]);
barg = bar(X, err_reduction);
set(gca, 'FontSize', 14)
ylabel('Error Reduction (%)', 'FontName', 'Arial', 'FontSize', 16);
barg(1).FaceColor = [0.3010 0.7450 0.9330];
barg(2).FaceColor = [0.93,0.69,0.13];
l2 = legend({'Average', 'Maximum'});
set(l2, 'Position', [0.6806 0.78 0.2067 0.1329])
ax = gca;
ax.XGrid = 'off';
ax.YGrid = 'on';
% set(gcf, 'Renderer', 'Painters')
% print(fig2, './Figure/percentage_err_plot', '-depsc')

fig3 = figure('Position', [200, 200, 1050, 420]);
H1 = subplot(1, 2, 1);
bar(X, average_err);
set(gca, 'FontSize', 15)
ylabel('Composite Error Norm Avg. (px)', 'FontName', 'Arial', 'FontSize', 17);
l1 = legend({'Distance Sum Method', 'Task-Oriented Metric'});
set(l1, 'Position', [0.2469 0.848 0.2181 0.1202])
ax = gca;
ax.XGrid = 'off';
ax.YGrid = 'on';

err_reduction(:, 1) = error_reduction_cc;
H2 = subplot(1, 2, 2);
barg = bar(X, err_reduction);
set(gca, 'FontSize', 15)
ylabel('Error Reduction Rate (%)', 'FontName', 'Arial', 'FontSize', 17);
barg(1).FaceColor = [0.3010 0.7450 0.9330];
barg(2).FaceColor = [0.93,0.69,0.13];
l2 = legend({'Average', 'Maximum'});
set(l2, 'Position', [0.78 0.8 0.1181 0.11])
ax = gca;
ax.XGrid = 'off';
ax.YGrid = 'on';
ylim([0, 63]);
set(gcf, 'Renderer', 'Painters')
print(fig3, './Figure/combined_bar_plot_update', '-depsc')


%% 
function [error_reduction_vec, reduction_rate] = computeReductionRatebyCase(bench_mark_data, method_data)
    data_num = size(bench_mark_data, 2);
    error_reduction_vec = zeros(1, data_num);
    reduction_rate = 0;
    for i = 1 : data_num
        error_reduction_vec(1, i) = (bench_mark_data(1, i) - method_data(1, i)) / bench_mark_data(1, i) * 100;
        reduction_rate = reduction_rate + error_reduction_vec(1, i);
    end
    reduction_rate = reduction_rate / data_num;
end