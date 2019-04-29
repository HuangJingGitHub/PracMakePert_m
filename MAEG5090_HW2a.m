%%% MAEG5090 Topics in Robotics Assignment1: Walking Control 2a %%%
clear
close all
clc

robotMass  = 80;
gacl       = 9.8;
CoMHeight  = 1.2;
maxStride  = 0.8;
footprint_x = 0.15;
footprint_y = 0.2;
maxFootprintDist = 0.6;

d_len = 0.01;
xy_ref(2,:) = 0:d_len:40;
xy_ref(1,:) = 3 * sin(pi / 20 * xy_ref(2,:));
sampleNum = length(xy_ref(2,:));
distance = 0.25;     % distance between the footprint and the CoM reference

ds = 0;  % Arc length
segError = 0.02;
stepLength = 0.5;
labelIndex = 1;
index = zeros(1, 200);
for i = 1:sampleNum
    slope = -1 / (3 * pi / 20 * cos( pi / 20 * xy_ref(2,i)));    
    ds = ds + sqrt(1 + 1/(slope)^2)*d_len;   % Integration for CoM curve length.
     if (stepLength - ds) < segError
         index(1, labelIndex) = i;
         labelIndex = labelIndex + 1;
         ds = 0;
     end
end
index(labelIndex:end) = [];   % In deleting matrix part, if colon apperas, only one index in the brckets is permitted.
stepNum = length(index);

footprintSer = zeros(4, stepNum);
for j = 1:stepNum
    i = index(j);
    slope = -1 / (3 * pi / 20 * cos( pi / 20 * xy_ref(2,i)));
    footPrintAngle = -atan(-1 / slope);  % Now y is the horizontal ordinate, x is the vertical ordinate. The rotz angle is negative.
    footPrintTime  = xy_ref(2,i) * 2;
    footprintSer(3, j) = footPrintAngle;
    footprintSer(4, j) = footPrintTime;
    if mod(j,2) == 1
        footprintSer(1:2, j) = xy_ref(:, i) + sign(slope) * distance / sqrt(1 + slope^2) * [slope; 1];
    else
        footprintSer(1:2, j) = xy_ref(:, i) - sign(slope) * distance / sqrt(1 + slope^2) * [slope; 1];
    end
end



figure('position',[120 95 1000 500])
plot(xy_ref(2,:), xy_ref(1,:));
hold on
plot(xy_ref(2,index), xy_ref(1, index),'Marker', 's', 'Color', 'b', 'LineWidth', 1.2)
% plot(footprintSer(2,:), footprintSer(1,:),'Marker', 's', 'Color', 'r')
footprintShape = [-footprint_x/2 footprint_x/2 footprint_x/2 -footprint_x/2;
                  -footprint_y/2 -footprint_y/2 footprint_y/2 footprint_y/2];
for i = 1:stepNum
    rotAng = footprintSer(3, i);   
    footprintShapeCur = footprintSer(1:2, i) * ones(1,4) + ...
                        [cos(rotAng) -sin(rotAng); sin(rotAng) cos(rotAng)] * footprintShape;
    if mod(i, 2) == 1
        line([footprintShapeCur(2, :), footprintShapeCur(2,1)], [footprintShapeCur(1, :), footprintShapeCur(1,1)],...
             'Color','r', 'LineWidth', 1.5);
    else
        line([footprintShapeCur(2, :), footprintShapeCur(2,1)], [footprintShapeCur(1, :), footprintShapeCur(1,1)],...
             'Color','g', 'LineWidth', 1.5);
    end
    hold on
end
axis equal
xlabel('y (m)','Fontname','Times', 'FontSize', 12);
ylabel('x (m)','Fontname','Times', 'FontSize', 12);

figure('position',[120 95 1000 500])
plot([0 footprintSer(4,1) footprintSer(4,1)], [0 0 1],'LineWidth', 1);
hold on
for i = 1:stepNum - 1
    Time1 = footprintSer(4, i);
    Time2 = footprintSer(4, i+1);
    if mod(i, 2) == 1
        line([Time1 Time2 Time2], [1 1 -1], 'LineWidth', 1);
    else
        line([Time1 Time2 Time2], [-1 -1 1], 'LineWidth', 1);
    end
    hold on
end
axis([0 80 -1.2 1.2])
xlabel('time (s)','Fontname','Times', 'FontSize', 12);
ylabel('left/right footprint','Fontname','Times', 'FontSize', 12);

%%% Give z_ref %%%
timeLen     = footprintSer(4, end);
dT    = 0.005;
timeSeries  = 0:dT:timeLen;
timeStepNum     = length(timeSeries);
z_ref = zeros(2, timeStepNum);
z_rot = zeros(1, timeStepNum);
j = 1;
for i = 1:timeStepNum
    time_cur = timeSeries(i);
    if time_cur <= footprintSer(4, j)
        z_ref(:, i) = footprintSer(1:2, j);
        z_rot(1, i) = footprintSer(3, j);
    else
        j = j+1;
        z_ref(:, i) = footprintSer(1:2, j);
        z_rot(1, i) = footprintSer(3, j);
    end
end
save('z_ref', 'z_ref');
save('z_rot', 'z_rot');
save('timeLen', 'timeLen');