clear all
close all
clc

ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);

load exampleMaps.mat
map = occupancyMap(simpleMap,10);
sv.Map = map;
sv.ValidationDistance = 0.01;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

planner = plannerRRTStar(ss,sv);
planner.ContinueAfterGoalReached = true;
planner.MaxIterations = 2500;
planner.MaxConnectionDistance = 0.3;

start = [0.5, 0.5 0];
goal = [2.5, 0.2, 0];
rng(10, 'twister') % repeatable result
[pthObj, solnInfo] = plan(planner, start, goal);

map.show;
hold on;
plot(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2), '.-'); % tree expansion
plot(pthObj.States(:,1),pthObj.States(:,2),'r-','LineWidth',2); % draw path