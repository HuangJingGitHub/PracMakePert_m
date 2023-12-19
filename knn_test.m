clc
clear all

treeNode = rand(10000, 2) * 100;
newNode = [50, 50];

tic;
minDistSqr = 1e7;
minDistIdx = 1;
for i = 1 : size(treeNode, 1)
    distVec = newNode - treeNode(i, :);
    curDistSqr = distVec * distVec.';
    if curDistSqr < minDistSqr
        minDistIdx = i;
        minDistSqr = curDistSqr;
    end
end
toc;
disp('Using loop iteration to find the nearest neighbor is: ')
disp(minDistIdx)

tic;
    knnMinDistIdx = knnsearch(treeNode, newNode);
toc;
disp('Using built-in knn function to find the nearest neighbor is: ')
disp(minDistIdx)
