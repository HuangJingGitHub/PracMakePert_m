%%
clear
close all

dirInfo = dir();
dt = 0.1;
%%
for i = 1 : size(dirInfo, 1)
    fileName = dirInfo(i).name;
    if (length(fileName) > 3 & fileName(end-2:end) == 'mat')
        load(fileName)
    end
end

ptErrorData = {pt_featurePt153, pt_featurePt191, pt_featurePt52, pt_featurePt75, pt_featurePt76};
ptErrorNorm = {zeros(1, length(pt_featurePt153)/2), zeros(1, length(pt_featurePt191)/2), zeros(1, length(pt_featurePt52)/2),...
             zeros(1, length(pt_featurePt75)/2), zeros(1, length(pt_featurePt76)/2)};
angleErrorData = {abs(angle165 - 45), abs(angle179 - 45), angle392, angle451, angle47};

for i = 1:length(ptErrorData)
    desiredPt = [ptErrorData{i}(end-1); ptErrorData{i}(end)];
    
    for j = 1:length(ptErrorData{i})/2
        ptErrorNorm{i}(1,j) = norm([ptErrorData{i}(2*j-1); ptErrorData{i}(2*j)] - desiredPt);
    end
end

%%
% figure
fig = figure('Position', [100 150 1200 420]);
H11 = subplot(1,2,1);
for i = 1:length(ptErrorNorm)
    plot(0:dt:dt*(length(ptErrorNorm{i})-1), ptErrorNorm{i}+5, 'LineWidth', 2)
    hold on
end
xlabel(['time (s)', newline, '\fontname{Times}\fontsize{22} (a)   '])
ylabel('||e_y||_2 (px)')
grid on

ax = gca;
ax.FontSize = 15;
% axis([0 25 0 80]);

H12 = subplot(1,2,2);
for i = 1:length(angleErrorData)
    plot(0:dt:dt*(length(angleErrorData{i})-1), angleErrorData{i}+1, 'LineWidth', 2);
    hold on
end
xlabel(['time (s)', newline, '\fontname{Times}\fontsize{22} (b)   '])
ylabel('|e_y| (deg)')
grid on

ax = gca;
ax.FontSize = 15;

% print(fig, './figure/Multi_error', '-depsc')