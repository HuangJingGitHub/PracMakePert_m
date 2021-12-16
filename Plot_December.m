clear
close all

expInfo = 'motion_interval: 45 ms motion_magnitude: 0.001000 m error_threshold: 5.000000 px';
dataFormatInfo = 'pt---target_pt---ee_pt---ee_to_obs_obs_pt---DO_to_obs_DO_pt---DO_to_obs_obs_pt---projection_on_path_pt---Jd';
singleDataDim = 18;
%% read file
fileID = fopen('planned_path_2021-7-7_23_38.txt', 'r');
pathData = fscanf(fileID, '%f');
pathSampleNum = size(pathData, 1) / 2;
pathDataXY = zeros(2, pathSampleNum);
for i = 1 : pathSampleNum
    pathDataXY(1, i) = pathData(2 * i - 1, 1);
    pathDataXY(2, i) = pathData(2 * i);
end
fclose(fileID);

fileID = fopen('singlept_data_2021-7-7_23_38.txt', 'r');
rawData = fscanf(fileID, '%f');
fclose(fileID);

fileID = fopen('local_path_width_2021-6-26_16_53.txt');
localPathWidth0626 = fscanf(fileID, '%f');
fclose(fileID);
fileID = fopen('planned_path_2021-6-26_16_53.txt', 'r');
pathData = fscanf(fileID, '%f');
pathSampleNum = size(pathData, 1) / 2;
pathDataXY0626 = zeros(2, pathSampleNum);
for i = 1 : pathSampleNum
    pathDataXY0626(1, i) = pathData(2 * i - 1, 1);
    pathDataXY0626(2, i) = pathData(2 * i);
end
fclose(fileID);

fileID = fopen('local_path_width_2021-7-7_23_38.txt');
localPathWidth0707 = fscanf(fileID, '%f');
fclose(fileID);
fileID = fopen('planned_path_2021-7-7_23_38.txt', 'r');
pathData = fscanf(fileID, '%f');
pathSampleNum = size(pathData, 1) / 2;
pathDataXY0707 = zeros(2, pathSampleNum);
for i = 1 : pathSampleNum
    pathDataXY0707(1, i) = pathData(2 * i - 1, 1);
    pathDataXY0707(2, i) = pathData(2 * i);
end
fclose(fileID);

%% format variables
dataNum = size(rawData, 1) / singleDataDim;
feedbackPt = zeros(2, dataNum);
eePt = zeros(2, dataNum);
DOToObsDOPt = zeros(2, dataNum);
DOToObsObsPt = zeros(2, dataNum);
projectionOnPathPt = zeros(2, dataNum);
Jd = zeros(2, dataNum * 2);
obsVertices = [297, 314, 310, 292; ...
               151, 150, 235, 235];

targetPt = [rawData(3, 1); rawData(4, 1)];
for i = 1 : dataNum
    ptDataIdx = i;
    rawDataIdx = singleDataDim * (i - 1);
    
    feedbackPt(1, ptDataIdx) = rawData(rawDataIdx + 1, 1);
    feedbackPt(2, ptDataIdx) = rawData(rawDataIdx + 2, 1);
    eePt(1, ptDataIdx) = rawData(rawDataIdx + 5, 1);
    eePt(2, ptDataIdx) = rawData(rawDataIdx + 6, 1);
    DOToObsDOPt(1, ptDataIdx) = rawData(rawDataIdx + 9, 1);
    DOToObsDOPt(2, ptDataIdx) = rawData(rawDataIdx + 10, 1);
    DOToObsObsPt(1, ptDataIdx) = rawData(rawDataIdx + 11, 1);
    DOToObsObsPt(2, ptDataIdx) = rawData(rawDataIdx + 12, 1);
    projectionOnPathPt(1, ptDataIdx) = rawData(rawDataIdx + 13, 1);
    projectionOnPathPt(2, ptDataIdx) = rawData(rawDataIdx + 14, 1);
    Jd(1, ptDataIdx * 2 - 1) = rawData(rawDataIdx + 15, 1);
    Jd(1, ptDataIdx * 2) = rawData(rawDataIdx + 16, 1);
    Jd(2, ptDataIdx * 2 - 1) = rawData(rawDataIdx + 17, 1);
    Jd(2, ptDataIdx * 2) = rawData(rawDataIdx + 18, 1);
end

%% data analysis
accumulatedPathLength = zeros(pathSampleNum, 1);
for i = 2 : pathSampleNum
    pathSample = pathDataXY(:, i);
    prePathSample = pathDataXY(:, i - 1);
    accumulatedPathLength(i, 1) = accumulatedPathLength(i - 1) + norm(pathSample - prePathSample);
end

pathLengthTracking = zeros(dataNum, 1);
absDisToTarget = zeros(dataNum, 1);
closestDisToPath = zeros(dataNum, 1);
forwardShift = fix(pathSampleNum / 20);
coordinateError = 0.1;
for i = 1 : dataNum
    curProjectionPt = projectionOnPathPt(:, i);
    projectionPtPathIdx = 0;
    for idx = 1 : pathSampleNum
        if abs(pathData(2 * idx - 1, 1) - curProjectionPt(1, 1)) < coordinateError ...
            && abs(pathData(2 * idx, 1) - curProjectionPt(2, 1)) < coordinateError
           projectionPtPathIdx = idx;
           break;
        end       
    end
    pathLengthTracking(i, 1) = accumulatedPathLength(end) - accumulatedPathLength(projectionPtPathIdx, 1);
    absDisToTarget(i, 1) = norm(feedbackPt(:, i) - targetPt);
    closestDisToPath(i, 1) = closestDistoPathFunc(feedbackPt(:, i), pathDataXY);
end

ptToPathProjection = zeros(dataNum, 1);
eeToObsDistance = zeros(dataNum, 1);
DOToObsDistance = zeros(dataNum, 1);
enterIdx = [];
leaveIdx = [];
for i = 1 : dataNum
    ptToPathProjection(i, 1) = norm(feedbackPt(:, i) - projectionOnPathPt(:, i));
    DOToObsDistance(i, 1) = norm(DOToObsDOPt(:, i) - DOToObsObsPt(:, i));
    eeToObsDistance(i, 1) = calPtDisToPolygon(eePt(:, i), obsVertices);
    if i > 1
        if eeToObsDistance(i, 1) < 0.5 && eeToObsDistance(i - 1, 1) > 0.5
            enterIdx = [enterIdx i];
        end
        if eeToObsDistance(i, 1) > 0.5 && eeToObsDistance(i - 1, 1) < 0.5
            leaveIdx = [leaveIdx i];
        end
    end    
end
enterIdx = min(enterIdx);
leaveIdx = max(leaveIdx);
for i = enterIdx : leaveIdx
    eeToObsDistance(i) = 0;
end

timeSeries = (1 : dataNum) * 0.045;

%% plot unitedly
files0626 = dir('./Mat/*.mat');
addpath 'C:\Users\thuhj\Files\Research\Constrained-DOM\Scripts\Data Processing\20210707\23_38\Mat'
for i = 1 : length(files0626)
    load(files0626(i).name)
end

figN = figure('position', [400 400 1460 520]);
H11 = subplot(2,4,1);
set(H11, 'Position', [0.07 0.6122 0.1566 0.3128])
plot(timeSeries0626, ptToPathProjection0626, 'Linewidth', 2)
hold on
plot(timeSeries0626, closestDisToPath0626, 'Linewidth', 2)
xTickObj = get(gca, 'XTickLabel');
set(gca, 'XTickLabel', xTickObj, 'Fontsize', 11)
ylabel('Path Err. (px)', 'Fontname', 'Times', 'Fontsize', 15)
xlabel('(a)', 'Fontname', 'Times', 'Fontsize', 15)
legend('Tracking error', 'Dist. to path', 'Fontname', 'Times', 'Fontsize', 13, 'Position', [0.125 0.86 0.1222 0.0890])
%set(gca, 'XTicklabel', [])
%grid on
xlim([0 23])

H12 = subplot(2,4,2);
set(H12, 'Position', [0.2961 0.6122 0.1566 0.3128])
plot(timeSeries0626, pathLengthTracking0626, 'Linewidth', 2)
hold on
plot(timeSeries0626, absDisToTarget0626, 'LineWidth', 2)
xTickObj = get(gca, 'XTickLabel');
set(gca, 'XTickLabel', xTickObj, 'Fontsize', 11)
ylabel('Target Err. (px)', 'Fontname', 'Times', 'Fontsize', 15)
xlabel('(b)', 'Fontname', 'Times', 'Fontsize', 15)
legend('In planned path', 'In configuration space', 'Fontname', 'Times', 'Fontsize', 13, 'Location', 'southwest')
%set(gca, 'XTicklabel', []);
%grid on
xlim([0 23])
ylim([0 256])

H13 = subplot(2,4,3);
set(H13, 'Position', [0.5222 0.6122 0.1566 0.3128])
plot(timeSeries0626, DOToObsDistance0626, 'Linewidth', 2)
hold on
plot(timeSeries0626, eeToObsDistance0626, 'Linewidth', 2)
xTickObj = get(gca, 'XTickLabel');
set(gca, 'XTickLabel', xTickObj, 'Fontsize', 11)
xlabel('(c)', 'Fontname', 'Times', 'Fontsize', 15)
ylabel('Obstacle Dist. (px)', 'Fontname', 'Times', 'Fontsize', 15)
legendObj = legend('DO-obstacle', 'ee-obstacle', 'Fontname', 'Times', 'Fontsize', 13, 'Position', [0.78 0.8182 0.0952 0.0890]);
%grid on
xlim([0 23])

H21 = subplot(2,4,5);
set(H21, 'Position', [0.0700 0.1383 0.1566 0.3128])
plot(timeSeries, ptToPathProjection, 'Linewidth', 2)
hold on
plot(timeSeries, closestDisToPath, 'Linewidth', 2)
adjustXBox = [3.5 15.5 28; 3.5  15.5 28];
adjustYBox = [0 0 0; 44.15 44.15 44.15];
hold on
plot(adjustXBox, adjustYBox, 'Color', 'r', 'Linewidth', 2)
% patch(adjustXBox, adjustYBox, 'black', 'FaceColor', 'b', 'FaceAlpha', 0.15)
% adjustXBox = [15.5 28 28 15.5];
% adjustYBox = [0 0 44.15 44.15];
% hold on
% patch(adjustXBox, adjustYBox, 'black', 'FaceColor', 'm', 'FaceAlpha', 0.15)
text(4, 28, {'Adjustment'; '    phase'}, 'Fontname', 'Arial', 'Fontsize', 13)
text(16.5, 10, {'Tracking'; ' EE path'}, 'Fontname', 'Arial', 'Fontsize', 13)

%set(gca, 'XTicklabel', [])
xTickObj = get(gca, 'XTickLabel');
set(gca, 'XTickLabel', xTickObj, 'Fontsize', 11)
ylabel('Path Err. (px)', 'Fontname', 'Times', 'Fontsize', 15)
xlabel({'Time (s)'; '(e)'}, 'Fontname', 'Times', 'Fontsize', 15)
% legend('Tracking error norm', 'Dist. to path', 'Fontname', 'Times', 'Fontsize', 13)
%grid on
xlim([0 38])
ylim([0 44.15])


H22 = subplot(2,4,6);
set(H22, 'Position', [0.2961 0.1383 0.1566 0.3128])
plot(timeSeries, pathLengthTracking, 'Linewidth', 2)
hold on
plot(timeSeries, absDisToTarget, 'LineWidth', 2)
adjustXBox = [3.5 15.5 28; 3.5  15.5 28];
adjustYBox = [0 0 0; 200 200 200];
hold on
plot(adjustXBox, adjustYBox, 'Color', 'r', 'Linewidth', 2)
% patch(adjustXBox, adjustYBox, 'black', 'FaceColor', 'b', 'FaceAlpha', 0.15)
% adjustXBox = [15.5 28 28 15.5];
% adjustYBox = [0 0 200 200];
% hold on
% patch(adjustXBox, adjustYBox, 'black', 'FaceColor', 'm', 'FaceAlpha', 0.15)

xTickObj = get(gca, 'XTickLabel');
set(gca, 'XTickLabel', xTickObj, 'Fontsize', 11)
ylabel('Target Err. (px)', 'Fontname', 'Times', 'Fontsize', 15)
xlabel({'Time (s)'; '(f)'}, 'Fontname', 'Times', 'Fontsize', 15)
% legend('In planned path', 'In configuration space', 'Fontname', 'Times', 'Fontsize', 13)
%set(gca, 'XTicklabel', []);
%grid on
xlim([0 38])

H23 = subplot(2,4,7);
set(H23, 'Position', [0.5222 0.1383 0.1566 0.3128])
plot(timeSeries, DOToObsDistance, 'Linewidth', 2)
hold on
plot(timeSeries, eeToObsDistance, 'Linewidth', 2)
adjustXBox = [3.5 15.5 28; 3.5  15.5 28];
adjustYBox = [0 0 0; 150 150 150];
hold on
plot(adjustXBox, adjustYBox, 'Color', 'r', 'Linewidth', 2)
% patch(adjustXBox, adjustYBox, 'black', 'FaceColor', 'b', 'FaceAlpha', 0.15)
% adjustXBox = [15.5 28 28 15.5];
% adjustYBox = [0 0 150 150];
% hold on
% patch(adjustXBox, adjustYBox, 'black', 'FaceColor', 'm', 'FaceAlpha', 0.15)

xTickObj = get(gca, 'XTickLabel');
set(gca, 'XTickLabel', xTickObj, 'Fontsize', 11)
xlabel({'Time (s)'; '(g)'}, 'Fontname', 'Times', 'Fontsize', 15)
ylabel('Obstacle Dist. (px)', 'Fontname', 'Times', 'Fontsize', 15)
% legend('DO-obstacle', 'ee-obstacle', 'Fontname', 'Times', 'Fontsize', 13);
%grid on
xlim([0 38])

H14 = subplot(2, 4, 4);
% set(H14, 'Position', [0.7484 0.6023 0.16 0.33])
plot3(pathDataXY0626(1, :), pathDataXY0626(2, :), ones(1, size(pathDataXY0626, 2)), '--m', 'LineWidth', 2);
hold on
plot3(pathDataXY0626(1, :), pathDataXY0626(2, :), localPathWidth0626, 'color', [0.8500 0.3250 0.0980], 'LineWidth', 2);
hold on
for i = 1 : length(localPathWidth0626)
    line([pathDataXY0626(1, i), pathDataXY0626(1, i)], [pathDataXY0626(2, i), pathDataXY0626(2, i)], [0, localPathWidth0626(i, 1)], 'LineWidth', 1);
end

xlabel({'(d)'}, 'Fontname', 'Times', 'Fontsize', 15)
%ylabel('y (px)', 'Fontname', 'Times', 'Fontsize', 15)
zlabel('Path with (px)', 'Fontname', 'Times', 'Fontsize', 15)
ax = gca;
ax.FontName = 'Times';
set(ax, 'Ydir', 'reverse')
grid on
% axis equal
axis([100 400 220 380 0 700])
view([-10 19.74])
xticks(100:50:400)
%yticks(220:40:380)

H24 = subplot(2, 4, 8);
%set(H24, 'Position', [0.7484 0.124 0.1566 0.3250])
plot3(pathDataXY0707(1, :), pathDataXY0707(2, :), ones(1, size(pathDataXY0707, 2)), '--m', 'LineWidth', 2);
hold on
plot3(pathDataXY0707(1, :), pathDataXY0707(2, :), localPathWidth0707, 'color', [0.8500 0.3250 0.0980], 'LineWidth', 2);
hold on
for i = 1 : length(localPathWidth0707)
    line([pathDataXY0707(1, i), pathDataXY0707(1, i)], [pathDataXY0707(2, i), pathDataXY0707(2, i)], [0 localPathWidth0707(i, 1)], 'LineWidth', 1);
end
xlabel({'x (px)'; '  (h)'}, 'Fontname', 'Times', 'Fontsize', 15)
%ylabel('y (px)', 'Fontname', 'Times', 'Fontsize', 15)
text(130, 250, -80, {'y (px)'}, 'Fontname', 'Times', 'Fontsize', 15)
zlabel('Path with (px)', 'Fontname', 'Times', 'Fontsize', 15)
ax = gca;
% ax.FontSize = 16;
ax.FontName = 'Times';
set(ax, 'Ydir', 'reverse')
grid on
%axis equal
axis([200 400 200 320 0 600])
view([-10 19.74])
% xticks(200:50:400)
% yticks(200:50:320)
set(gcf, 'Renderer', 'Painters');
print(figN, './Figure/DataAnalysis_0626_0707_All_in_One', '-depsc')


%% path width
% fig1 = figure('position', [500 500 820 368]);
% H11 = subplot(1, 2, 1);
% set(H11, 'Position', [0.100 0.1200 0.360 0.8150])
% plot3(pathDataXY0626(1, :), pathDataXY0626(2, :), ones(1, size(pathDataXY0626, 2)), '--m', 'LineWidth', 2);
% hold on
% plot3(pathDataXY0626(1, :), pathDataXY0626(2, :), localPathWidth0626, 'color', [0.8500 0.3250 0.0980], 'LineWidth', 2);
% hold on
% for i = 1 : length(localPathWidth0626)
%     line([pathDataXY0626(1, i), pathDataXY0626(1, i)], [pathDataXY0626(2, i), pathDataXY0626(2, i)], [0, localPathWidth0626(i, 1)], 'LineWidth', 1);
% end
% 
% xlab = xlabel('x (px)','Fontname','Times', 'Fontsize', 16);
% set(xlab, 'Position', [250 405 -50])
% ylab = ylabel('y (px)', 'Fontname', 'Times', 'Fontsize', 16);
% set(ylab, 'Position', [48 300 -40])
% zlabel('Local Path Width (px)', 'Fontname', 'Times', 'Fontsize', 16)
% ax = gca;
% ax.FontSize = 16;
% ax.FontName = 'Times';
% set(ax, 'Ydir', 'reverse')
% grid on
% % axis equal
% axis([100 400 220 380 0 700])
% view([347.6 32.8])
% xticks(100:60:400)
% yticks(220:40:380)
% t1 = title('(a)', 'Fontname', 'Times', 'Fontsize', 20, 'FontWeight','Normal', 'Position', [276, 235, 666]);
% 
% H12 = subplot(1, 2, 2);
% set(H12, 'Position', [0.5703 0.1200 0.360 0.8150])
% plot3(pathDataXY0707(1, :), pathDataXY0707(2, :), ones(1, size(pathDataXY0707, 2)), '--m', 'LineWidth', 2);
% hold on
% plot3(pathDataXY0707(1, :), pathDataXY0707(2, :), localPathWidth0707, 'color', [0.8500 0.3250 0.0980], 'LineWidth', 2);
% hold on
% for i = 1 : length(localPathWidth0707)
%     line([pathDataXY0707(1, i), pathDataXY0707(1, i)], [pathDataXY0707(2, i), pathDataXY0707(2, i)], [0 localPathWidth0707(i, 1)], 'LineWidth', 1);
% end
% % xlab = xlabel('x (px)','Fontname','Times', 'Fontsize', 14);
% % set(xlab, 'Position', [300 320 -81])
% % ylab = ylabel('y (px)', 'Fontname', 'Times', 'Fontsize', 14);
% % set(ylab, 'Position', [185 250 -80])
% % zlabel('Local Path Width (px)', 'Fontname', 'Times', 'Fontsize', 14)
% ax = gca;
% ax.FontSize = 16;
% ax.FontName = 'Times';
% set(ax, 'Ydir', 'reverse')
% grid on
% %axis equal
% axis([200 400 200 320 0 600])
% view([347.6 32.8])
% xticks(200:40:400)
% yticks(200:40:320)
% t2 = title('(b)', 'Fontname', 'Times', 'Fontsize', 20, 'FontWeight','Normal', 'Position', [317 212 576]);
% set(gcf, 'Renderer', 'Painters');