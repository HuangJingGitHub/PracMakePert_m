%%
clear
close all

dirInfo = dir();
dataList = zeros();

for i = 1 : size(dirInfo, 1)
    fileName = dirInfo(i).name;
    fileNameLen = length(dirInfo(i).name);
    if (fileNameLen >= 17 & fileName(1:17) == 'disturb_featurePt')
        load(fileName)
        dataList = [dataList, "", fileName(1:end-4)];
    end
end
errorData = {pt_featurePt153, pt_featurePt191, pt_featurePt52, pt_featurePt75, pt_featurePt76};
errorNorm = {zeros(1, length(pt_featurePt153)/2), zeros(1, length(pt_featurePt191)/2), zeros(1, length(pt_featurePt52)/2),...
             zeros(1, length(pt_featurePt75)/2), zeros(1, length(pt_featurePt76)/2)};

for i = 1:length(errorData)
    desiredPt = [errorData{i}(end-1); errorData{i}(end)];
    
    for j = 1:length(errorData{i})/2
        errorNorm{i}(1,j) = norm([errorData{i}(2*j-1); errorData{i}(2*j)] - desiredPt);
    end
end

% figure 
% plot(errorNorm{5})
%%
% figure
for i = 1:5
    plot(errorNorm{i}, 'LineWidth', 2)
    hold on
end

