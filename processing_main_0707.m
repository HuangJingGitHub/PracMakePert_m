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
    DOToObsObsPt(1, ptDataIdx) = rawData(rawDataIdx + 9, 1);
    DOToObsObsPt(2, ptDataIdx) = rawData(rawDataIdx + 10, 1);
    DOToObsDOPt(1, ptDataIdx) = rawData(rawDataIdx + 11, 1);
    DOToObsDOPt(2, ptDataIdx) = rawData(rawDataIdx + 12, 1);
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
    pathSample = [pathData(2 * i - 1, 1); pathData(2 * i)];
    prePathSample = [pathData(2 * i - 3, 1); pathData(2 * i - 2)];
    accumulatedPathLength(i, 1) = accumulatedPathLength(i - 1) + norm(pathSample - prePathSample);
end

pathLengthTracking = zeros(dataNum, 1);
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
end

ptToPathProjection = zeros(dataNum, 1);
DOToObsDistance = zeros(dataNum, 1);
for i = 1 : dataNum
    ptToPathProjection(i, 1) = norm(feedbackPt(:, i) - projectionOnPathPt(:, i));
    DOToObsDistance(i, 1) = norm(DOToObsDOPt(:, i) - DOToObsObsPt(:, i)); 
end

%% plot
timeSeries = (1 : dataNum) * 0.045;
fig1 = figure('position', [100 50 700 520]);
plot(pathDataXY(1, :), pathDataXY(2,:), '--m', 'Linewidth', 2)
hold on
plot(feedbackPt(1, :), feedbackPt(2, :), 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 2)
hold on
plot(pathDataXY(1,1), pathDataXY(2,1), '-o','MarkerSize',10,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor','r')
hold on
plot(pathDataXY(1,end), pathDataXY(2,end), '-o','MarkerSize',10,...
    'MarkerEdgeColor','g',...
    'MarkerFaceColor','g')
legend('Path Reference', 'Real Path', 'Location', 'northwest')

xlabel('x (px)','Fontname','Times','Fontsize',14);
ylabel('y (px)', 'Fontname', 'Times','Fontsize', 14);
ax = gca;
ax.FontSize = 14;
ax.FontName = 'Times';
set(ax, 'Ydir', 'reverse')
grid on
axis equal
axis([200 400 200 320])
% xticks(250:10:310)
% yticks(200:10:250)
%print(fig1, './figure/SignlePtTrajectory_0501_464', '-depsc')

fig2 = figure('position', [741.6667 -280.3333 868.0000 852.6667]);
H11 = subplot(3,1,1);
plot(timeSeries, pathLengthTracking, 'color', 'b', 'Linewidth', 2)
ylabel('Path Diatance to Target (px)', 'Fontname', 'Times', 'Fontsize', 12)
%set(gca, 'XTicklabel', []);
grid on
gca.FontSize = 16;
gca.FontName = 'Times';
% axis([0 22 0 80])

H12 = subplot(3,1,2);
plot(timeSeries, ptToPathProjection, 'color', 'b', 'Linewidth', 2)
ylabel('Feedback to Path (px)', 'Fontname', 'Times', 'Fontsize', 12)
%set(gca, 'XTicklabel', [])
grid on
gca.FontSize = 16;
gca.FontName = 'Times';
% axis([0 22 15 30])

H13 = subplot(3,1,3);
plot(timeSeries, DOToObsDistance, 'color', 'b', 'Linewidth', 2)
xlabel('Time (s)', 'Fontname', 'Times', 'Fontsize', 12)
ylabel('DO-Obstacle (px)', 'Fontname', 'Times', 'Fontsize', 12)
gca.FontSize = 16;
gca.FontName = 'Times';
% axis([0 22 0 20])
grid on
%print(fig2, './figure/SignlePt_error_L_d_0501_464', '-depsc')