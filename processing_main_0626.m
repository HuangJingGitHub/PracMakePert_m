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

%% format variables
dataNum = size(rawData, 1) / singleDataDim;
feedbackPt = zeros(2, dataNum);
eePt = zeros(2, dataNum);
DOToObsDOPt = zeros(2, dataNum);
DOToObsObsPt = zeros(2, dataNum);
projectionOnPathPt = zeros(2, dataNum);
Jd = zeros(2, dataNum * 2);

targetPt = [rawData(3, 1); rawData(4, 1)];
for i = 1 : dataNum
    ptDataIdx = i;
    rawDataIdx = singleDataDim * (i - 1);
    
    feedbackPt(1, ptDataIdx) = rawData(rawDataIdx + 1, 1);
    feedbackPt(2, ptDataIdx) = rawData(rawDataIdx + 2, 1);
    eePt(1, ptDataIdx) = rawData(rawDataIdx + 5, 1);
    eePt(2, ptDataIdx) = rawData(rawDataIdx + 6, 1);
    DOToObsObsPt(1, ptDataIdx) = rawData(rawDataIdx + 7, 1);
    DOToObsObsPt(2, ptDataIdx) = rawData(rawDataIdx + 8, 1);
    DOToObsDOPt(1, ptDataIdx) = rawData(rawDataIdx + 9, 1);
    DOToObsDOPt(2, ptDataIdx) = rawData(rawDataIdx + 10, 1);
    projectionOnPathPt(1, ptDataIdx) = rawData(rawDataIdx + 11, 1);
    projectionOnPathPt(2, ptDataIdx) = rawData(rawDataIdx + 12, 1);
    Jd(1, ptDataIdx * 2 - 1) = rawData(rawDataIdx + 13, 1);
    Jd(1, ptDataIdx * 2) = rawData(rawDataIdx + 14, 1);
    Jd(2, ptDataIdx * 2 - 1) = rawData(rawDataIdx + 15, 1);
    Jd(2, ptDataIdx * 2) = rawData(rawDataIdx + 16, 1);
end

%% data analysis
accumulatedPathLength = zeros(pathSampleNum, 1);
for i = 2 : pathSampleNum
    pathSample = [pathData(2 * i - 1, 1); pathData(2 * i)];
    prePathSample = [pathData(2 * i - 3, 1); pathData(2 * i - 2)];
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
    %closestDisToPath(i, 1) = norm(feedbackPt(:, i) - pathDataXY(:, projectionPtPathIdx - forwardShift));
    closestDisToPath(i, 1) = closestDistoPathFunc(feedbackPt(:, i), pathDataXY);
end

ptToPathProjection = zeros(dataNum, 1);
DOToObsDistance = zeros(dataNum, 1);
for i = 1 : dataNum
    ptToPathProjection(i, 1) = norm(feedbackPt(:, i) - projectionOnPathPt(:, i));
    DOToObsDistance(i, 1) = norm(DOToObsDOPt(:, i) - DOToObsObsPt(:, i)); 
end

timeSeries = (1 : dataNum) * 0.045;
%% plot 1
% fig1 = figure('position', [709 356 700 400]);
% plot(pathDataXY(1, :), pathDataXY(2,:), '--m', 'Linewidth', 2)
% hold on
% plot(feedbackPt(1, :), feedbackPt(2, :), 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2)
% hold on
% plot(pathDataXY(1,1), pathDataXY(2,1), '-o','MarkerSize',10,...
%     'MarkerEdgeColor','r',...
%     'MarkerFaceColor','r')
% hold on
% plot(pathDataXY(1,end), pathDataXY(2,end), '-o','MarkerSize',10,...
%     'MarkerEdgeColor','g',...
%     'MarkerFaceColor','g')
% text(pathDataXY(1,1) - 1.8, pathDataXY(2, 1) + 9, '\uparrow', 'Fontsize', 14)
% text(pathDataXY(1,1) - 7.5, pathDataXY(2, 1) + 17, 'start', 'Fontsize', 14)
% text(pathDataXY(1,end) - 1.8, pathDataXY(2, end) - 11, '\downarrow', 'Fontsize', 14)
% text(pathDataXY(1,end) - 9, pathDataXY(2, end) - 18, 'target', 'Fontsize', 14)
% legend('Reference Path', 'Real Path', 'Location', 'Northwest')
% 
% xlabel('x (px)','Fontname','Times','Fontsize', 14);
% ylabel('y (px)', 'Fontname', 'Times','Fontsize', 14);
% ax = gca;
% ax.FontSize = 14;
% ax.FontName = 'Times';
% set(ax, 'Ydir', 'reverse')
% grid on
% axis equal
% axis([120 400 220 380])
% xticks(120:40:400)
% yticks(220:40:380)
% print(fig1, './Figure/PathTracking_16_53', '-depsc')

%% plot 2
% fig2 = figure('position', [955 93 868 852]);
fig2 = figure('position', [955 144 774 720]);
H11 = subplot(3,1,1);
plot(timeSeries, pathLengthTracking, 'color', 'b', 'Linewidth', 2)
hold on
plot(timeSeries, absDisToTarget, 'LineWidth', 2)
ylabel('Target Err. (px)', 'Fontname', 'Times', 'Fontsize', 14)
xlabel('(a)', 'Fontname', 'Times', 'Fontsize', 14)
%set(gca, 'XTicklabel', []);
grid on
gca.FontSize = 18;
gca.FontName = 'Times';
xlim([0 23])
ylim([0 256])


H12 = subplot(3,1,2);
plot(timeSeries, ptToPathProjection, 'color', 'b', 'Linewidth', 2)
hold on
plot(timeSeries, closestDisToPath, 'Linewidth', 2)
ylabel('Path Err. (px)', 'Fontname', 'Times', 'Fontsize', 14)
xlabel('(b)', 'Fontname', 'Times', 'Fontsize', 14)
%set(gca, 'XTicklabel', [])
grid on
gca.FontSize = 18;
gca.FontName = 'Times';
xlim([0 23])

H13 = subplot(3,1,3);
plot(timeSeries, DOToObsDistance, 'color', 'b', 'Linewidth', 2)
xlabel({'Time (s)'; '(c)'}, 'Fontname', 'Times', 'Fontsize', 14)
ylabel('Obstacle Dis. (px)', 'Fontname', 'Times', 'Fontsize', 14)
gca.FontSize = 18;
gca.FontName = 'Times';
xlim([0 23])
grid on
print(fig2, './Figure/DataAnalysis_16_53_label', '-depsc')
