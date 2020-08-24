%% Load raw data
clear
close all
load('disturb_featurePt75.mat')
load('disturb_contact_distance75.mat')
load('disturb_manipulability75.mat')

%% 
featurePt = pt_featurePt75;
localContact = pt_contact_distance75;
manipulabilityIdx = pt_manipulability75;

timeStep = 0.1;
time = 0 : timeStep : timeStep*(length(localContact)-1);
startPt = featurePt(1:2)';
targetPt = startPt - [50; 40];

paraAngle = 0: 0.05 : 2*pi;
radious = 5;
targetRegionx = targetPt(1,1) + radious * cos(paraAngle);
targetRegiony = targetPt(2,1) + radious * sin(paraAngle);

featurePtTrajectory = zeros(2, length(featurePt)/2);
posError = zeros(1, length(featurePt)/2);
for i = 1 : length(featurePt)/2
    featurePtTrajectory(1, i) = featurePt(2*i-1);
    featurePtTrajectory(2, i) = featurePt(2*i);
    posError(i) = norm(featurePtTrajectory(:, i) - targetPt);
end


%% plot
fig2 = figure('Position', [100 100 1120 380]);
H11 = subplot(1,2,1);
plot(time, posError, 'color', 'b', 'Linewidth', 2)
hold on
plot([6.8 6.8], [0, 70], 'Color', [.929 .694 .125], 'Linewidth',2)
hold on
plot([15.5 15.5], [0, 70], 'Color', [.929 .694 .125], 'Linewidth',2)
xlabel('time (s)')
ylabel('||e_y||_2 (px)')
%set(gca, 'XTicklabel', []);
grid on
gca.FontSize = 15;
axis([0 25 0 70])
txt = {'\leftarrow local contact', '     disturbance'};
text(6.8, 18,txt, 'fontsize', 12)

H12 = subplot(1,2,2);
plot(time, localContact, 'Color', 'b', 'Linewidth', 2)
hold on
plot([0 time(end)], [20 20], '--r', 'Linewidth', 2)
hold on
plot([6.8 6.8], [0, 40], 'Color', [.929 .694 .125], 'Linewidth',2)
hold on
plot([15.5 15.5], [0, 40], 'Color', [.929 .694 .125], 'Linewidth',2)

xlabel('time (s)')
ylabel('L (px)')
grid on
ax = gca;
ax.FontSize = 15;
% ax.XLim = [0 25];
% ax.YLim = [0 40];
axis([0 25 0 40])
txt = {'\leftarrow local contact', '     disturbance'};
text(6.8, 11,txt, 'fontsize', 12)

% print(fig2, './figure/DisturbanceData0525', '-depsc')