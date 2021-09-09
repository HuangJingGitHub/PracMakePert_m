clear
close all

expInfo = 'motion_interval: 45 ms motion_magnitude: 0.000500 m position_threshold: 5.000000 px angle_error_threshold 5 deg';
dataFormatInfo = 'Format: feedback_pt---target_pt---ee_pt---DO_to_obs_DO_pt---DO_to_obs_obs_pt---projection_on_path_pt---Jd---target_angle---cur_angle';
singleDataDim = 38;
feedbackPtNum = 3;

%% read file
fileID = fopen('angle_data_2021-9-3_13_12.txt', 'r');
rawData1 = fscanf(fileID, '%f');
fclose(fileID);

%% format variables
dataNum1 = size(rawData1, 1) / singleDataDim;
feedbackPt1 = zeros(6, dataNum1);
eePt1 = zeros(2, dataNum1);
DOToObsDOPt1 = zeros(2, dataNum1);
DOToObsObsPt1 = zeros(2, dataNum1);
projectionOnPathPt1 = zeros(6, dataNum1);
Jd1 = zeros(6, dataNum1 * 2);
curAngle1 = zeros(1, dataNum1);
obsVertices = [338, 388, 374, 331; ...
               158, 165, 5, 6]; 

targetPt1 = zeros(6, 1);
for i = 7 : 12
    targetPt1(i - 6, 1) = rawData1(i, 1);
end
targetAngle = rawData1(37, 1);

for i = 1 : dataNum1
    ptDataIdx = i;
    rawDataIdx = singleDataDim * (i - 1);
    
    for j = 1 : 6
        feedbackPt1(j, ptDataIdx) = rawData1(rawDataIdx + j, 1);
    end
    eePt1(1, ptDataIdx) = rawData1(rawDataIdx + 13, 1);
    eePt1(2, ptDataIdx) = rawData1(rawDataIdx + 14, 1);
    DOToObsDOPt1(1, ptDataIdx) = rawData1(rawDataIdx + 15, 1);
    DOToObsDOPt1(2, ptDataIdx) = rawData1(rawDataIdx + 16, 1);
    DOToObsObsPt1(1, ptDataIdx) = rawData1(rawDataIdx + 17, 1);
    DOToObsObsPt1(2, ptDataIdx) = rawData1(rawDataIdx + 18, 1);
    for j = 1 : 6
        projectionOnPathPt1(j, ptDataIdx) = rawData1(rawDataIdx + 18 + j, 1);
    end
    
    cnt = 1;
    for row = 1 : 6
        for col = 1 : 2
        Jd1(row, 2 * (ptDataIdx - 1) + col) = rawData1(rawDataIdx + 24 + cnt, 1);
        cnt = cnt + 1;
        end
    end
    curAngle1(1, i) = rawData1(rawDataIdx + singleDataDim); 
end

%% data analysis
eeToObsDistance1 = zeros(1, dataNum1);
DOToObsDistance1 = zeros(1, dataNum1);
enterIdx = [];
leaveIdx = [];
for i = 1 : dataNum1
    DOToObsDistance1(1, i) = norm(DOToObsDOPt1(:, i) - DOToObsObsPt1(:, i));
    if DOToObsDistance1(1, i) < 5
        DOToObsDistance1(1, i) = DOToObsDistance1(1, i - 1);
    end
    eeToObsDistance1(1, i) = calPtDisToPolygon(eePt1(:, i), obsVertices);
    if i > 1
        if eeToObsDistance1(1, i) < 0.5 && eeToObsDistance1(1, i - 1) > 0.5
            enterIdx = [enterIdx i];
        end
        if eeToObsDistance1(1, i) > 0.5 && eeToObsDistance1(1, i - 1) < 0.5
            leaveIdx = [leaveIdx i];
        end
    end    
end
enterIdx = min(enterIdx);
leaveIdx = max(leaveIdx);
for i = enterIdx : leaveIdx
    eeToObsDistance1(i) = 0;
end

%% read file 2
fileID = fopen('angle_data_2021-9-3_13_45.txt', 'r');
rawData2 = fscanf(fileID, '%f');
fclose(fileID);

%% format variables
dataNum2 = size(rawData2, 1) / singleDataDim;
feedbackPt2 = zeros(6, dataNum2);
eePt2 = zeros(2, dataNum2);
DOToObsDOPt2 = zeros(2, dataNum2);
DOToObsObsPt2 = zeros(2, dataNum2);
projectionOnPathPt2 = zeros(6, dataNum2);
Jd2 = zeros(6, dataNum2 * 2);
curAngle2 = zeros(1, dataNum2);
obsVertices2 = [338, 388, 374, 331; ...
               158, 165, 5, 6]; 

targetPt2 = zeros(6, 1);
for i = 7 : 12
    targetPt2(i - 6, 1) = rawData2(i, 1);
end
targetAngle = rawData1(37, 1);

for i = 1 : dataNum2
    ptDataIdx = i;
    rawDataIdx = singleDataDim * (i - 1);
    
    for j = 1 : 6
        feedbackPt2(j, ptDataIdx) = rawData2(rawDataIdx + j, 1);
    end
    eePt2(1, ptDataIdx) = rawData2(rawDataIdx + 13, 1);
    eePt2(2, ptDataIdx) = rawData2(rawDataIdx + 14, 1);
    DOToObsDOPt2(1, ptDataIdx) = rawData2(rawDataIdx + 15, 1);
    DOToObsDOPt2(2, ptDataIdx) = rawData2(rawDataIdx + 16, 1);
    DOToObsObsPt2(1, ptDataIdx) = rawData2(rawDataIdx + 17, 1);
    DOToObsObsPt2(2, ptDataIdx) = rawData2(rawDataIdx + 18, 1);
    for j = 1 : 6
        projectionOnPathPt2(j, ptDataIdx) = rawData2(rawDataIdx + 18 + j, 1);
    end
    
    cnt = 1;
    for row = 1 : 6
        for col = 1 : 2
        Jd2(row, 2 * (ptDataIdx - 1) + col) = rawData2(rawDataIdx + 24 + cnt, 1);
        cnt = cnt + 1;
        end
    end
    curAngle2(1, i) = rawData2(rawDataIdx + singleDataDim); 
end

%% data analysis
eeToObsDistance2 = zeros(1, dataNum2);
DOToObsDistance2 = zeros(1, dataNum2);
pivotDis2 = zeros(1, dataNum2);
enterIdx = [];
leaveIdx = [];
for i = 1 : dataNum2
    DOToObsDistance2(1, i) = norm(DOToObsDOPt2(:, i) - DOToObsObsPt2(:, i));
    if DOToObsDistance2(1, i) > 100
        DOToObsDistance2(1, i) = DOToObsDistance2(1, i - 1);
    end
    eeToObsDistance2(1, i) = calPtDisToPolygon(eePt2(:, i), obsVertices);
    pivotDis2(1, i) = norm(feedbackPt2(1:2, i) - targetPt2(1:2, 1));
    if i > 1
        if eeToObsDistance2(1, i) < 0.5 && eeToObsDistance2(1, i - 1) > 0.5
            enterIdx = [enterIdx i];
        end
        if eeToObsDistance2(1, i) > 0.5 && eeToObsDistance2(1, i - 1) < 0.5
            leaveIdx = [leaveIdx i];
        end
    end    
end
enterIdx = min(enterIdx);
leaveIdx = max(leaveIdx);
for i = enterIdx : leaveIdx
    eeToObsDistance1(i) = 0;
end

timeSeries1 = (1 : dataNum1) * 0.045;
timeSeries2 = (1 : dataNum2) * 0.045;
%% plot 1
fig1 = figure('position', [955 144 764 460]);
H11 = subplot(2,1,1);
plot(timeSeries1, curAngle1, 'Linewidth', 2)
hold on
plot([0, 25], [121, 121], '--r', 'LineWidth', 2)
xTickObj = get(gca, 'XTickLabel');
set(gca, 'XTickLabel', xTickObj, 'Fontsize', 11)
ylabel('\alpha (deg)', 'Fontname', 'Times', 'Fontsize', 15)
xlabel('(a)', 'Fontname', 'Times', 'Fontsize', 15)
grid on
%xlim([0 34])
ylim([60 180])
yticks(60:40:180)

H21 = subplot(2,1,2);
plot(timeSeries1, DOToObsDistance1, 'Linewidth', 2)
xTickObj = get(gca, 'XTickLabel');
set(gca, 'XTickLabel', xTickObj, 'Fontsize', 11)
ylabel('DO-Obs. Dist. (px)', 'Fontname', 'Times', 'Fontsize', 15)
xlabel({'Time (s)'; '(b)'}, 'Fontname', 'Times', 'Fontsize', 15)
grid on
set(gcf, 'Renderer', 'Painters');
print(fig1, './Figure/Comparison_0903_Angle_DotoObsDis_Tracking', '-depsc')

%% plot 2
fig2 = figure('position', [955 144 764 460]);
H11 = subplot(2,1,1);
plot(timeSeries2(1:770), curAngle2(1:770), 'LineWidth', 2)
hold on
plot([0, 40], [121, 121], '--r', 'LineWidth', 2)
xTickObj = get(gca, 'XTickLabel');
set(gca, 'XTickLabel', xTickObj, 'Fontsize', 11)
ylabel('\alpha (deg)', 'Fontname', 'Times', 'Fontsize', 15)
xlabel('(a)', 'Fontname', 'Times', 'Fontsize', 15)
grid on
xlim([0 35])
ylim([60 180])
yticks(60:40:180)

H21 = subplot(2,1,2);
plot(timeSeries2(1:770), DOToObsDistance2(1:770), 'LineWidth', 2)
xTickObj = get(gca, 'XTickLabel');
set(gca, 'XTickLabel', xTickObj, 'Fontsize', 11)
ylabel('DO-Obs. Dist. (px)', 'Fontname', 'Times', 'Fontsize', 15)
xlabel({'Time (s)'; '(b)'}, 'Fontname', 'Times', 'Fontsize', 15)
grid on
xlim([0 35])
ylim([0 65])
set(gcf, 'Renderer', 'Painters');
print(fig2, './Figure/Comparison_0903_Angle_DotoObsDis_Error_Driven', '-depsc')