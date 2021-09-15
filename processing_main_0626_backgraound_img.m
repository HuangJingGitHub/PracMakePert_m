clear
close all

expInfo = 'motion_interval: 45 ms motion_magnitude: 0.001000 m error_threshold: 5.000000 px';
dataFormatInfo = 'Format: feedback_pt---target_pt---ee_pt---DO_to_obs_DO_pt---DO_to_obs_obs_pt---projection_on_path_pt---Jd';
singleDataDim = 16;
%% read file
fileID = fopen('planned_path_2021-6-26_16_53.txt', 'r');
pathData = fscanf(fileID, '%f');
pathSampleNum = size(pathData, 1) / 2;
pathDataXY = zeros(2, pathSampleNum);
for i = 1 : pathSampleNum
    pathDataXY(1, i) = pathData(2 * i - 1, 1);
    pathDataXY(2, i) = pathData(2 * i);
end
fclose(fileID);

fileID = fopen('singlept_data_2021-6-26_16_53.txt', 'r');
rawData = fscanf(fileID, '%f');
fclose(fileID);

fileID = fopen('local_path_width_2021-6-26_16_53.txt');
localPathWidth = fscanf(fileID, '%f');
fclose(fileID);
%% format variables
dataNum = size(rawData, 1) / singleDataDim;
feedbackPt = zeros(2, dataNum);
eePt = zeros(2, dataNum);
DOToObsDOPt = zeros(2, dataNum);
DOToObsObsPt = zeros(2, dataNum);
projectionOnPathPt = zeros(2, dataNum);
Jd = zeros(2, dataNum * 2);
obsVertices = [236, 254, 244, 226; ...
               229, 229, 297, 300];

targetPt = [rawData(3, 1); rawData(4, 1)];
for i = 1 : dataNum
    ptDataIdx = i;
    rawDataIdx = singleDataDim * (i - 1);
    
    feedbackPt(1, ptDataIdx) = rawData(rawDataIdx + 1, 1);
    feedbackPt(2, ptDataIdx) = rawData(rawDataIdx + 2, 1);
    eePt(1, ptDataIdx) = rawData(rawDataIdx + 5, 1);
    eePt(2, ptDataIdx) = rawData(rawDataIdx + 6, 1);
    DOToObsDOPt(1, ptDataIdx) = rawData(rawDataIdx + 7, 1);
    DOToObsDOPt(2, ptDataIdx) = rawData(rawDataIdx + 8, 1);
    DOToObsObsPt(1, ptDataIdx) = rawData(rawDataIdx + 9, 1);
    DOToObsObsPt(2, ptDataIdx) = rawData(rawDataIdx + 10, 1);
    projectionOnPathPt(1, ptDataIdx) = rawData(rawDataIdx + 11, 1);
    projectionOnPathPt(2, ptDataIdx) = rawData(rawDataIdx + 12, 1);
    Jd(1, ptDataIdx * 2 - 1) = rawData(rawDataIdx + 13, 1);
    Jd(1, ptDataIdx * 2) = rawData(rawDataIdx + 14, 1);
    Jd(2, ptDataIdx * 2 - 1) = rawData(rawDataIdx + 15, 1);
    Jd(2, ptDataIdx * 2) = rawData(rawDataIdx + 16, 1);
end


timeSeries = (1 : dataNum) * 0.045;
%% plot 1
fig1 = figure('position', [709 356 500 380]);
img = imread("./Background Images/exp_snip_0626_0_500x380.png");
image('CData', img, 'XData', [70, 570], 'YData', [100, 480])
hold on
plot(pathDataXY(1, :), pathDataXY(2,:), '--m', 'Linewidth', 2)
hold on
% plot(feedbackPt(1, :), feedbackPt(2, :), 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2)
plot(feedbackPt(1, :), feedbackPt(2, :), 'Color', 'g', 'LineWidth', 2)
hold on
startPt = plot(pathDataXY(1,1), pathDataXY(2,1), '-o','MarkerSize', 6,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor','b');
startPt.Annotation.LegendInformation.IconDisplayStyle = 'off';
hold on
targetPt = plot(pathDataXY(1,end), pathDataXY(2,end), '-o','MarkerSize', 6,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor','g');
targetPt.Annotation.LegendInformation.IconDisplayStyle = 'off';
hold on
patch(obsVertices(1, :), obsVertices(2, :),'b', 'FaceColor', 'b', 'FaceAlpha', 0.4)
legend('Reference Path', 'Real Path', 'Obstacle', 'Fontsize', 16, 'Position', [0.134 0.7 0.3620 0.2026])
text(pathDataXY(1,1) - 8, pathDataXY(2, 1) + 17, 'start', 'Fontname', 'Arial', 'Fontsize', 16, 'Color', [1, 1, 1])
text(pathDataXY(1,end) - 25, pathDataXY(2, end) + 18, 'target', 'Fontname', 'Arial', 'Fontsize', 16, 'Color', [1, 1, 1])

%xlabel('x (px)','Fontname','Times','Fontsize', 14);
%ylabel('y (px)', 'Fontname', 'Times','Fontsize', 14);
ax = gca;
ax.FontSize = 14;
ax.FontName = 'Times';
set(ax, 'Ydir', 'reverse')
grid on
axis equal
axis([70 570 100 480])
xticks([])
yticks([])
set(gcf, 'Renderer', 'Painters');
print(fig1, './Figure/PathTracking_Obs_0626_1653_backgraund_img_initial', '-depsc')
