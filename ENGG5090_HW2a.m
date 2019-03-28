%%% MAEG5090 Topics in Robotics Assignment1: Walking Control 2a %%%
clear
close all
clc

robotMass  = 80;
CoMHeight  = 1.2;
maxStride  = 0.8;
footprint_x = 0.15;
footprint_y = 0.2;
footprintDist = 0.6;

timeLen     = 80;
timeStep    = 0.005;
timeSeries  = 0:timeStep:timeLen;
stepNum     = length(timeSeries);

xy_ref(2,:) = 0:0.01:40;
xy_ref(1,:) = 3 * sin(pi / 20 * xy_ref(2,:));
sampleNum = length(xy_ref(2,:));
left_ref = zeros(2, sampleNum);
right_ref = zeros(2, sampleNum);
distance = 0.25;
ds = 0;  % 
labelIndex = 1;
for i = 1:sampleNum
    slope = -1 / (3 * pi / 20 * cos( pi / 20 * xy_ref(2,i)));
%     cos_temp = 1 / sqrt(1 + slope^2);
%     sin_temp = abs(slope) / sqrt(1 + slope^2);
%     left_ref(:, i) = [ xy_ref(1,i) + distance * sin_temp; xy_ref(2,i) + sign(slope) * distance * cos_temp];
     %left_ref(:, i) = xy_ref(:, i) + sign(slope) * distance / sqrt(1 + slope^2) * [slope; 1] ;
     %right_ref(:, i) = xy_ref(:, i) - sign(slope) * distance / sqrt(1 + slope^2) * [slope; 1] ;
     
     ds = ds + sqrt(1 + 1/(slope)^2)*0.01;
     if (1 - ds) < 0.05
         index(1, labelIndex) = i;
         labelIndex = labelIndex + 1;
         ds = 0;
     end
end

footPrints = zeros(2, length(index));
for j = 1:length(index)
    i = index(j);
    slope = -1 / (3 * pi / 20 * cos( pi / 20 * xy_ref(2,i)));
    if mod(j,2) == 1
    footPrints(:, j) = xy_ref(:, i) + sign(slope) * distance / sqrt(1 + slope^2) * [slope; 1] ;
    else
    footPrints(:, j) = xy_ref(:, i) - sign(slope) * distance / sqrt(1 + slope^2) * [slope; 1] ;
    end
end

figure
plot(xy_ref(2,:), xy_ref(1,:));
hold on
% plot(left_ref(2,:), left_ref(1,:));
% hold on
% plot(right_ref(2,:), right_ref(1,:));
hold on
plot(xy_ref(2,index), xy_ref(1, index),'Marker', 's', 'Color', 'b')
%axis equal
% plot(y_ref, x_ref_r);
plot(footPrints(2,:), footPrints(1,:),'Marker', 's', 'Color', 'r')
