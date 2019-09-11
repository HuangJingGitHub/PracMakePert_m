%% process one selected group data in 20090905 as the representation 
%% of Jacobian based planner
%% Load data
close all
load('rotate_angle_30deg_2.mat')
load('centroid_30deg_2.mat')
rotateAngle = angle_30deg_2;
centroid_raw = centroid_30deg_2;
rotateAngle(54) = [];
centroid_raw(107:108) = [];
stepNumber = length(rotateAngle);

load('rotate_angle_30deg_1_mp.mat') % data of rotation-based motion planner
load('centroid_30deg_1_mp.mat')
rotateAngle_mp = angle_30deg_1_mp;
centroid_raw_mp = centroid_30deg_1_mp;
stepNumber_mp = length(rotateAngle_mp);
%% Get coordinate
centroid = zeros(2, stepNumber);
for i = 1 : stepNumber
    centroid(1, i) = centroid_raw(2*i-1);
    centroid(2, i) = centroid_raw(2*i);
end

centroid_mp = zeros(2, stepNumber_mp);
for i = 1 : stepNumber_mp
    centroid_mp(1, i) = centroid_raw_mp(2*i-1);
    centroid_mp(2, i) = centroid_raw_mp(2*i);
end

%% Use mean of the centroid to represend target position
targetPositionData = centroid(:,(stepNumber-4):end);
targetPosition = sum(targetPositionData, 2) / 5;

targetPositionData_mp = centroid_mp(:,(stepNumber_mp-4):end);
targetPosition_mp = sum(targetPositionData_mp, 2) / 5;

%% Get error
targetAngle = 30;
errorAngle = targetAngle - rotateAngle;
errorCentroid = zeros(2, stepNumber);
errorCentroidNorm = zeros(1, stepNumber);
for i = 1 : stepNumber
    errorCentroid(:, i) = targetPosition - centroid(:,i);
    errorCentroidNorm(i) = norm(errorCentroid(:, i));
end

targetAngle_mp = 30;
errorAngle_mp = targetAngle_mp - rotateAngle_mp;
errorCentroid_mp = zeros(2, stepNumber_mp);
errorCentroidNorm_mp = zeros(1, stepNumber_mp);
for i = 1 : stepNumber_mp
    errorCentroid_mp(:, i) = targetPosition_mp - centroid_mp(:,i);
    errorCentroidNorm_mp(i) = norm(errorCentroid_mp(:, i));
end

timeSeries = 0 : 0.1 : 0.1 * (stepNumber - 1);
timeSeries_mp = 0 : 0.1 : 0.1 * (stepNumber_mp - 1);
%% Plot
figure('position', [100 50 850 600]);
H11 = subplot(2,1,1);
plot(timeSeries, errorCentroidNorm, 'Linewidth', 2);
hold on
plot(timeSeries_mp, errorCentroidNorm_mp, 'Color', 'r', 'Linewidth', 2)
ylabel('{\bf ||\Delta s_c||_2}  (px)', 'Fontname', 'Times','Fontsize', 17);
set(gca,'XTicklabel',[]); 
legend({'Single-point Contact Control', 'Rotational Motion Planner'}, 'Fontname', 'Times', 'Fontsize', 17)
grid on
yticks(0:10:60)
ax = gca;
%set(gca,'linewidth',1);
%ax.GridAlpha = 1;
ax.FontSize = 16;
ax.FontName = 'Times';

H12 = subplot(2,1,2);
plot(timeSeries, errorAngle, 'Linewidth', 2);
hold on
plot(timeSeries_mp, errorAngle_mp, 'color', 'r', 'Linewidth', 2)
ylabel('\Delta\theta (deg)', 'Fontname', 'Times','Fontsize', 17);
xlabel('Time (s)','Fontname','Times','Fontsize',18);
legend({'Single-point Contact Control', 'Rotational Motion Planner'}, 'Fontname', 'Times', 'Fontsize', 17)
ax = gca;
%set(gca,'linewidth',1);
%ax.GridAlpha = 1;
ax.FontSize = 16;
ax.FontName = 'Times';
grid on



