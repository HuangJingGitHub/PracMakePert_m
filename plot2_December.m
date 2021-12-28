clear
close all

expInfo = 'motion_interval: 45 ms motion_magnitude: 0.000500 m position_threshold: 5.000000 px angle_error_threshold 5 deg';
dataFormatInfo = 'Format: feedback_pt---target_pt---ee_pt---DO_to_obs_DO_pt---DO_to_obs_obs_pt---projection_on_path_pt---Jd---target_angle---cur_angle';
singleDataDim = 38;
feedbackPtNum = 3;

%% read file
fileID = fopen('planned_path_pt_1_2021-7-29_21_29.txt', 'r');
pathData = fscanf(fileID, '%f');
pathSampleNum = size(pathData, 1) / 2;
pathDataXYPt1 = zeros(2, pathSampleNum);
for i = 1 : pathSampleNum
    pathDataXYPt1(1, i) = pathData(2 * i - 1, 1);
    pathDataXYPt1(2, i) = pathData(2 * i, 1);
end
fclose(fileID);
fileID = fopen('planned_path_pt_2_2021-7-29_21_29.txt', 'r');
pathData = fscanf(fileID, '%f');
pathSampleNum = size(pathData, 1) / 2;
pathDataXYPt2 = zeros(2, pathSampleNum);
for i = 1 : pathSampleNum
    pathDataXYPt2(1, i) = pathData(2 * i - 1, 1);
    pathDataXYPt2(2, i) = pathData(2 * i, 1);
end
fclose(fileID);
fileID = fopen('planned_path_pt_3_2021-7-29_21_29.txt', 'r');
pathData = fscanf(fileID, '%f');
pathSampleNum = size(pathData, 1) / 2;
pathDataXYPt3 = zeros(2, pathSampleNum);
for i = 1 : pathSampleNum
    pathDataXYPt3(1, i) = pathData(2 * i - 1, 1);
    pathDataXYPt3(2, i) = pathData(2 * i, 1);
end
fclose(fileID);

fileID = fopen('angle_data_2021-7-29_21_29.txt', 'r');
rawData = fscanf(fileID, '%f');
fclose(fileID);

% fileID = fopen('local_path_width_2021-7-7_23_38.txt');
% localPathWidth = fscanf(fileID, '%f');
% fclose(fileID);

%% format variables
dataNum = size(rawData, 1) / singleDataDim;
feedbackPt = zeros(6, dataNum);
eePt = zeros(2, dataNum);
DOToObsDOPt = zeros(2, dataNum);
DOToObsObsPt = zeros(2, dataNum);
projectionOnPathPt = zeros(6, dataNum);
Jd = zeros(6, dataNum * 2);
curAngle = zeros(1, dataNum);
obsVertices = [197.5, 154.5, 335.5, 383.5, 371.5, 332.5, 182.5, 212.5; ...
               270.5,   222,   110.5, 142.5, 151.5, 129, 225,   262]; %197 271 155 222 335 111 384 143 371 151 332 129 183 225 212 262 

targetPt = zeros(6, 1);
for i = 7 : 12
    targetPt(i - 6, 1) = rawData(i, 1);
end
targetAngle = rawData(37, 1);

for i = 1 : dataNum
    ptDataIdx = i;
    rawDataIdx = singleDataDim * (i - 1);
    
    for j = 1 : 6
        feedbackPt(j, ptDataIdx) = rawData(rawDataIdx + j, 1);
    end
    eePt(1, ptDataIdx) = rawData(rawDataIdx + 13, 1);
    eePt(2, ptDataIdx) = rawData(rawDataIdx + 14, 1);
    DOToObsDOPt(1, ptDataIdx) = rawData(rawDataIdx + 15, 1);
    DOToObsDOPt(2, ptDataIdx) = rawData(rawDataIdx + 16, 1);
    DOToObsObsPt(1, ptDataIdx) = rawData(rawDataIdx + 17, 1);
    DOToObsObsPt(2, ptDataIdx) = rawData(rawDataIdx + 18, 1);
    for j = 1 : 6
        projectionOnPathPt(j, ptDataIdx) = rawData(rawDataIdx + 18 + j, 1);
    end
    
    cnt = 1;
    for row = 1 : 6
        for col = 1 : 2
        Jd(row, 2 * (ptDataIdx - 1) + col) = rawData(rawDataIdx + 24 + cnt, 1);
        cnt = cnt + 1;
        end
    end
    curAngle(1, i) = rawData(rawDataIdx + singleDataDim); 
end

%% data analysis
accumulatedPathLengthPt1 = zeros(size(pathDataXYPt1, 2), 1);
for i = 2 : size(pathDataXYPt1, 2)
    pathSample = pathDataXYPt1(:, i);
    prePathSample = pathDataXYPt1(:, i - 1);
    accumulatedPathLengthPt1(i, 1) = accumulatedPathLengthPt1(i - 1, 1) + norm(pathSample - prePathSample);
end
accumulatedPathLengthPt2 = zeros(size(pathDataXYPt2, 2), 1);
for i = 2 : size(pathDataXYPt2, 2)
    pathSample = pathDataXYPt2(:, i);
    prePathSample = pathDataXYPt2(:, i - 1);
    accumulatedPathLengthPt2(i, 1) = accumulatedPathLengthPt2(i - 1, 1) + norm(pathSample - prePathSample);
end
accumulatedPathLengthPt3 = zeros(size(pathDataXYPt3, 2), 1);
for i = 2 : size(pathDataXYPt3, 2)
    pathSample = pathDataXYPt3(:, i);
    prePathSample = pathDataXYPt3(:, i - 1);
    accumulatedPathLengthPt3(i, 1) = accumulatedPathLengthPt3(i - 1, 1) + norm(pathSample - prePathSample);
end

pathLengthTracking = zeros(3, dataNum);
absDisToTarget = zeros(3, dataNum);
closestDisToPath = zeros(3, dataNum);
coordinateError = 0.1;
for i = 1 : dataNum
    curProjectionPt1 = projectionOnPathPt(1:2, i);
    projectionPtPathIdx = 1;
    for idx = 1 : size(pathDataXYPt1, 2)
        if abs(pathDataXYPt1(1, idx) - curProjectionPt1(1, 1)) < coordinateError ...
            && abs(pathDataXYPt1(2, idx) - curProjectionPt1(2, 1)) < coordinateError
           projectionPtPathIdx = idx;
           break;
        end       
    end
    pathLengthTracking(1, i) = accumulatedPathLengthPt1(end) - accumulatedPathLengthPt1(projectionPtPathIdx, 1);
    absDisToTarget(1, i) = norm(feedbackPt(1:2, i) - targetPt(1:2));
    closestDisToPath(1, i) = closestDistoPathFunc(feedbackPt(1:2, i), pathDataXYPt1);
    
    curProjectionPt2 = projectionOnPathPt(3:4, i);
    projectionPtPathIdx = 1;
    for idx = 1 : size(pathDataXYPt2, 2)
        if abs(pathDataXYPt2(1, idx) - curProjectionPt2(1, 1)) < coordinateError ...
            && abs(pathDataXYPt2(2, idx) - curProjectionPt2(2, 1)) < coordinateError
           projectionPtPathIdx = idx;
           break;
        end       
    end
    pathLengthTracking(2, i) = accumulatedPathLengthPt2(end) - accumulatedPathLengthPt2(projectionPtPathIdx, 1);
    absDisToTarget(2, i) = norm(feedbackPt(3:4, i) - targetPt(3:4));
    closestDisToPath(2, i) = closestDistoPathFunc(feedbackPt(3:4, i), pathDataXYPt2);
    
    curProjectionPt3 = projectionOnPathPt(5:6, i);
    projectionPtPathIdx = 1;
    for idx = 1 : size(pathDataXYPt3, 2)
        if abs(pathDataXYPt3(1, idx) - curProjectionPt3(1, 1)) < coordinateError ...
            && abs(pathDataXYPt3(2, idx) - curProjectionPt3(2, 1)) < coordinateError
           projectionPtPathIdx = idx;
           break;
        end       
    end
    pathLengthTracking(3, i) = accumulatedPathLengthPt3(end) - accumulatedPathLengthPt3(projectionPtPathIdx, 1);
    absDisToTarget(3, i) = norm(feedbackPt(5:6, i) - targetPt(5:6));
    closestDisToPath(3, i) = closestDistoPathFunc(feedbackPt(5:6, i), pathDataXYPt3);       
end

ptToPathProjectionDis = zeros(3, dataNum);
eeToObsDistance = zeros(1, dataNum);
DOToObsDistance = zeros(1, dataNum);
enterIdx = [];
leaveIdx = [];
for i = 1 : dataNum
    ptToPathProjectionDis(1, i) = norm(feedbackPt(1:2, i) - projectionOnPathPt(1:2, i));
    ptToPathProjectionDis(2, i) = norm(feedbackPt(3:4, i) - projectionOnPathPt(3:4, i));
    ptToPathProjectionDis(3, i) = norm(feedbackPt(5:6, i) - projectionOnPathPt(5:6, i));
    DOToObsDistance(1, i) = norm(DOToObsDOPt(:, i) - DOToObsObsPt(:, i));
    eeToObsDistance(1, i) = calPtDisToPolygon(eePt(:, i), obsVertices);
    if i > 1
        if eeToObsDistance(1, i) < 0.5 && eeToObsDistance(1, i - 1) > 0.5
            enterIdx = [enterIdx i];
        end
        if eeToObsDistance(1, i) > 0.5 && eeToObsDistance(1, i - 1) < 0.5
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
%% plot 1
lambda = 0 : 0.2 : 1;
targetConfig = zeros(6, 6);
targetConfig(1, :) = [312, 180, 312, 246.22, 388.258, 166.554];
targetConfig(2, :) = [312, 180, 312, 246.22, 388.563, 166.5];
targetConfig(3, :) = [312, 180, 303.362, 248.379, 390.889, 176.142];
targetConfig(4, :) = [312, 180, 282.654, 242.363, 388.22, 200.708];
targetConfig(5, :) = [312, 180, 275.069, 238.193, 385.023, 210.098];
targetConfig(6, :) = [312, 180, 271.488, 235.76, 382.99, 214.624];

figN = figure('position', [880 610 800 320]);
H11 = subplot(1, 2, 1); 
set(H11, 'Position', [0.08 0.17 0.43 0.8])
patch(obsVertices(1, :), obsVertices(2, :), 'b', 'FaceColor', 'b', 'FaceAlpha', 0.4)
hold on
plot([pathDataXYPt1(1, 1), pathDataXYPt2(1, 1)], [pathDataXYPt1(2, 1), pathDataXYPt2(2, 1)], 'b', 'LineWidth', 2)
plot([pathDataXYPt1(1, 1), pathDataXYPt3(1, 1)], [pathDataXYPt1(2, 1), pathDataXYPt3(2, 1)], 'b', 'LineWidth', 2)
plot(pathDataXYPt1(1, 1), pathDataXYPt1(2, 1), '-o', 'MarkerSize', 8, 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b')
plot(pathDataXYPt2(1, 1), pathDataXYPt2(2, 1), '-o', 'MarkerSize', 8, 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b')
plot(pathDataXYPt3(1, 1), pathDataXYPt3(2, 1), '-o', 'MarkerSize', 8, 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b')

for i = 1 : 1 : 6
    hold on
    curPlot = plot([targetConfig(i, 1), targetConfig(i, 3)], [targetConfig(i, 2), targetConfig(i, 4)], 'LineWidth', 2);
    curColor = curPlot.Color;
    plot([targetConfig(i, 1), targetConfig(i, 5)], [targetConfig(i, 2), targetConfig(i, 6)], 'color', curColor, 'LineWidth', 2)
    plot(targetConfig(i, 3), targetConfig(i, 4), '-o', 'MarkerSize', 8, 'MarkerEdgeColor',curColor, 'MarkerFaceColor', curColor)
    plot(targetConfig(i, 5), targetConfig(i, 6), '-o', 'MarkerSize', 8, 'MarkerEdgeColor',curColor, 'MarkerFaceColor', curColor)
end
plot(targetConfig(1, 1), targetConfig(1, 2), '-o', 'MarkerSize', 8, 'MarkerEdgeColor', 'b', 'MarkerFaceColor', 'b')
% circular_arrow(fig4, 110, [312, 180], 0, 30, -1, 'r', 10)
% draw an arrow
arrow = annotation('arrow');
arrow.Parent = gca;
arrow.Position = [414.5 200 -0.05 -0.1];
arrow.Color = 'r';
center = [370; 180];
radius = 50;
arcPts = zeros(2, 50);
for i = 1 : 50
    curAngle = (-22.5 + 45 / 50 * i) / 180 * pi;
    arcPts(1, i) = center(1, 1) + radius * cos(curAngle);
    arcPts(2, i) = center(2, 1) + radius * sin(curAngle);
end
plot(arcPts(1, :), arcPts(2, :), 'r', 'LineWidth', 1.5);

text(pathDataXYPt1(1,1) - 18, pathDataXYPt1(2, 1) + 24, 's_0 and y_0', 'Fontname', 'Arial', 'Fontsize', 17)
text(436, 288, 'x (px)', 'Fontname', 'Times', 'Fontsize', 17)
yLabel = text(164, 150, 'y (px)', 'Fontname', 'Times', 'Fontsize', 17);
set(yLabel, 'Rotation', 90)
text(300, 333, '(a)', 'Fontname', 'Times', 'Fontsize', 17)
%xlabel('x (px)','Fontname','Times','Fontsize', 19);
%ylabel('y (px)', 'Fontname', 'Times','Fontsize', 19);
ax = gca;
ax.FontSize = 16;
ax.FontName = 'Times';
set(ax, 'Ydir', 'reverse')
grid on
box on
%axis equal
axis([150 500 100 300])
% xticks(200:40:400)
% yticks(200:40:320)


H12 = subplot(1, 2, 2);
set(H12, 'Position', [0.55 0.17 0.43 0.8])
plot(pathDataXYPt1(1, :), pathDataXYPt1(2,:), '--m', 'Linewidth', 2)
hold on
plot(feedbackPt(1, :), feedbackPt(2, :), 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2)
hold on
patch(obsVertices(1, :), obsVertices(2, :), 'b', 'FaceColor', 'b', 'FaceAlpha', 0.4)
for i = 1 : 25 : size(feedbackPt, 2)
    hold on
    line([feedbackPt(1, i), feedbackPt(3, i)], [feedbackPt(2, i), feedbackPt(4, i)], 'Color', 'b', 'LineWidth', 2)
    line([feedbackPt(1, i), feedbackPt(5, i)], [feedbackPt(2, i), feedbackPt(6, i)], 'Color', 'b', 'LineWidth', 2)
end
% hold on
% line([feedbackPt(1, end), feedbackPt(3, end)], [feedbackPt(2, end), feedbackPt(4, end)], 'Color', 'b', 'LineWidth', 2)
% line([feedbackPt(1, end), feedbackPt(5, end)], [feedbackPt(2, end), feedbackPt(6, end)], 'Color', 'b', 'LineWidth', 2)
% line([pathDataXYPt1(1, end), pathDataXYPt2(1, end)], [pathDataXYPt1(2, end), pathDataXYPt2(2, end)], 'Color', 'g', 'LineWidth', 2)
% line([pathDataXYPt1(1, end), pathDataXYPt3(1, end)], [pathDataXYPt1(2, end), pathDataXYPt3(2, end)], 'Color', 'g', 'LineWidth', 2)
hold on
startPt = plot(pathDataXYPt1(1,1), pathDataXYPt1(2,1), '-o', 'MarkerSize',8,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor','r');
startPt.Annotation.LegendInformation.IconDisplayStyle = 'off';
hold on
targetPt = plot(pathDataXYPt1(1,end), pathDataXYPt1(2,end), '-o', 'MarkerSize',8,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor','g');
targetPt.Annotation.LegendInformation.IconDisplayStyle = 'off';

hold on
plot(pathDataXYPt2(1, :), pathDataXYPt2(2,:), '--m', 'Linewidth', 2)
hold on
plot(feedbackPt(3, :), feedbackPt(4, :), 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2)
hold on
startPt = plot(pathDataXYPt2(1,1), pathDataXYPt2(2,1), '-o', 'MarkerSize',8,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor','r');
startPt.Annotation.LegendInformation.IconDisplayStyle = 'off';
hold on
targetPt = plot(pathDataXYPt2(1,end), pathDataXYPt2(2,end), '-o', 'MarkerSize',8,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor','g');
targetPt.Annotation.LegendInformation.IconDisplayStyle = 'off';

hold on
plot(pathDataXYPt3(1, :), pathDataXYPt3(2,:), '--m', 'Linewidth', 2)
hold on
plot(feedbackPt(5, :), feedbackPt(6, :), 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2)
hold on
startPt = plot(pathDataXYPt3(1,1), pathDataXYPt3(2,1), '-o', 'MarkerSize',8,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor','r');
startPt.Annotation.LegendInformation.IconDisplayStyle = 'off';
hold on
targetPt = plot(pathDataXYPt3(1,end), pathDataXYPt3(2,end), '-o', 'MarkerSize',8,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor','g');
targetPt.Annotation.LegendInformation.IconDisplayStyle = 'off';
% legend('Reference Path', 'Real Path', 'Obstacle', 'Fontsize', 16, 'Position', [0.56 0.678 0.1667 0.2150], 'Box', 'off')
text(pathDataXYPt1(1,1), pathDataXYPt1(2, 1) + 15, 's_1', 'Fontname', 'Arial', 'Fontsize', 17)
text(pathDataXYPt2(1,1), pathDataXYPt2(2, 1) + 15, 's_2', 'Fontname', 'Arial', 'Fontsize', 17)
text(pathDataXYPt3(1,1) - 20, pathDataXYPt3(2, 1) + 15, 's_3', 'Fontname', 'Arial', 'Fontsize', 17)

%xlabel('x (px)','Fontname','Times','Fontsize', 19);
%ylabel('y (px)', 'Fontname', 'Times','Fontsize', 19);
%text(436, 288, 'x (px)', 'Fontname', 'Times', 'Fontsize', 17)
text(300, 333, '(b)', 'Fontname', 'Times', 'Fontsize', 17)
set(gca, 'YTickLabel', [])
ax = gca;
ax.FontSize = 17;
ax.FontName = 'Times';
set(ax, 'Ydir', 'reverse')
grid on
%axis equal
axis([150 500 100 300])
set(gcf, 'Renderer', 'Painters');
print(figN, './Figure/Angle_Simulaiton_and_Path', '-depsc')