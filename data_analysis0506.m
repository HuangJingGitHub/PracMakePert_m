%% Load raw data
clear
close all

dirInfo = dir();
postfix = '451.mat';
postfixLen = length(postfix);

for i = 1 : size(dirInfo, 1)
    fileName = dirInfo(i).name;
    fileNameLen = length(dirInfo(i).name);
    if (fileNameLen > postfixLen) & (fileName(fileNameLen - postfixLen + 1: end) == postfix)
        load(fileName)
    end
end

timeStep = 0.1;
dataNum = length(angle_angle451);
time = 0 : timeStep : timeStep*(dataNum - 1);

%% trajectory of 3 points, the weighted point sw
s0Trajectory = zeros(2, dataNum);
s1Trajectory = zeros(2, dataNum);
s2Trajectory = zeros(2, dataNum);
sw = zeros(2, dataNum);
distanceLinewl = zeros(1, dataNum);
distanceLinewr = zeros(1, dataNum);
distanceLinewp = zeros(1, dataNum);
weight0 = zeros(1, dataNum);
weight1 = zeros(1, dataNum);
weight2 = zeros(1, dataNum);

for i = 1 : dataNum
    s0Trajectory(1, i) = angle_s3_451(6*i - 5);
    s0Trajectory(2, i) = angle_s3_451(6*i - 4);
    s1Trajectory(1, i) = angle_s3_451(6*i - 3);
    s1Trajectory(2, i) = angle_s3_451(6*i - 2);
    s2Trajectory(1, i) = angle_s3_451(6*i - 1);
    s2Trajectory(2, i) = angle_s3_451(6*i); 
    
    sw(1, i) = angle_sw451(2*i - 1);
    sw(2, i) = angle_sw451(2*i);
    
    distanceLinewl(1, i) = angle_distance_linelrp451(3*i - 2);
    distanceLinewr(1, i) = angle_distance_linelrp451(3*i - 1);
    distanceLinewp(1, i) = angle_distance_linelrp451(3*i);
    
    weight0(1, i) = angle_weightVector451(3*i - 2);
    weight1(1, i) = angle_weightVector451(3*i - 1);
    weight2(1, i) = angle_weightVector451(3*i);
end

%% figure 1
fig1 = figure('position', [100 50 700 520]);
plot(s0Trajectory(1,:), s0Trajectory(2,:), 'Linewidth', 2)
hold on
plot(s1Trajectory(1,:), s1Trajectory(2,:), 'Linewidth', 2)
xlabel('x (px)','Fontname','Times','Fontsize',18);
ylabel('y (px)', 'Fontname', 'Times','Fontsize', 18);
ax = gca;
ax.FontSize = 16;
ax.FontName = 'Times';
set(ax, 'Ydir', 'reverse')
grid on
axis equal
axis([170 310 240 300])
xticks(150:20:400)
yticks(220:20:320)
%print(fig1, './figure/AngleTrajectory_s3_0506_451', '-depsc')
