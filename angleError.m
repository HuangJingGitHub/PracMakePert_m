%% Load raw data
clear
close all

dirInfo = dir();
postfix = '.mat';
postfixLen = length(postfix);
dataList = zeros();

for i = 1 : size(dirInfo, 1)
    fileName = dirInfo(i).name;
    fileNameLen = length(dirInfo(i).name);
    if (fileNameLen > 3 & fileName(end-2:end) == 'mat')
        load(fileName)
        dataList = [dataList, "", fileName(1:end-4)];
    end
end

figure
plot(abs(angle165 - 45), 'LineWidth', 2)
hold on
plot(abs(angle179 - 45), 'LineWidth', 2)
hold on
plot(angle392, 'LineWidth', 2)
hold on
plot(angle451, 'LineWidth', 2)
hold on
plot(angle47, 'LineWidth', 2)
