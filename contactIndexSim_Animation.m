close all
clear

% 2D control points

controlPts = [0.20, 0.72, 2.20, 3.70, 5.16, 5.27, 4.74, 3.70, 2.00, -0.25, 0.20;
              0.50, 0.00, 1.20, 0.15, 0.70, 1.40, 1.95, 1.50, 2.50, 1.20, 0.50];
controlPts(1,:) = controlPts(1,:) * 0.95;
controlPts(2,:) = controlPts(2,:) * 1.05;

t = 0:0.02:1;
C = open_quadratic_bspline(controlPts,t);


%% simulation setting
% sacle to image space
controlPts = 100 * controlPts;
C = 100 * C;
% s = [175, 320, 300;
%      110, 110, 175];
% s = [180, 350, 312;
%      110, 112, 175];
s = [200, 350, 315;
     150, 108, 175];
% s = [320, 135, 168;
%      160, 107, 195];
p_length_1 = (max(C(1,:)) - min(C(1,:))) / 5;
p_length_2 = (max(C(1,:)) - min(C(1,:))) / 3-5;


%% calculate TCI in loop in first 3 curves
% steps = 220;
% plotIdx = 70;
steps = 150;
plotIdx = 60;

s_erMat = zeros(2, steps);
s_emMat = zeros(2, steps);
neMat = zeros(2, steps);
TCI = zeros(1, steps);
p_midX = zeros(1, steps);
for i = 1:steps
    s_el = C(:, i);
    [s_er, TCI(1, i)] = calcTCI(C, i, p_length_1, s);
    s_em = (s_el + s_er) / 2;
    p_midX(1, i) = s_em(1,1);
    p_direction = (s_er - s_el) / norm(s_er - s_el);
    ne_direction = [-p_direction(2,1); p_direction(1,1)];
    
    s_erMat(:, i) = s_er;
    s_emMat(:, i) = s_em;
    neMat(:, i) = ne_direction;
end


%%
% steps = 220;
% plotIdx = 70;
steps = 120;
plotIdx = 105;

s_erMat2 = zeros(2, steps);
s_emMat2 = zeros(2, steps);
neMat2 = zeros(2, steps);
TCI2 = zeros(1, steps);
p_midX2 = zeros(1, steps);
for i = 1:steps
    s_el = C(:, i);
    [s_er, TCI2(1, i)] = calcTCI(C, i, p_length_2, s);
    s_em = (s_el + s_er) / 2;
    p_midX2(1, i) = s_em(1,1);
    p_direction = (s_er - s_el) / norm(s_er - s_el);
    ne_direction = [-p_direction(2,1); p_direction(1,1)];
    
    s_erMat2(:, i) = s_er;
    s_emMat2(:, i) = s_em;
    neMat2(:, i) = ne_direction;
end


%% figure
fig1 = figure('position', [200 200 1200 600]);
h1 = subplot(1,2,1);
% control points of contour
% plot(controlPts(1,:), controlPts(2,:), '-o', 'MarkerSize', 5, 'MarkerEdgeColor','r', ...
%     'MarkerFaceColor','r')
% hold on
% angle feature points
plot([s(1,1), s(1,2)], [s(2,1), s(2,2)], '-o', 'Color', 'k', 'Linewidth', 2, ...
    'MarkerSize', 5, 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
hold on
plot([s(1,1), s(1,3)], [s(2,1), s(2,3)], '-o', 'Color', 'g', 'LineWidth', 2, ...
    'MarkerSize', 5, 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
hold on
text(s(1,1)-15, s(2,1)-10, 's_0', 'FontSize', 15);
text(s(1,2)+10, s(2,2)-10, 's_1', 'FontSize', 15);
text(s(1,3)+10, s(2,3)-10, 's_2', 'FontSize', 15);
% contour
plot(C(1,:), C(2,:), 'Color', 'b', 'LineWidth', 2.5)
hold on 
% p representation
p_line = line([C(1, plotIdx), s_erMat(1,plotIdx)], [C(2, plotIdx), s_erMat(2, plotIdx)], 'Marker', 's', 'Color', 'r', 'Linewidth', 2, ...
        'MarkerSize', 5, 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
% textSl = text(C(1, plotIdx)-10, C(2, plotIdx)-15, 's_{el}', 'FontSize', 15);
% textSr = text(s_erMat(1, plotIdx)-10, s_erMat(2, plotIdx)-15, 's_{er}', 'FontSize', 15);
% textNe = text(s_emMat(1, plotIdx), s_emMat(2, plotIdx)+35, 'n_e', 'FontSize', 15);
neArrow = quiver(s_emMat(1, plotIdx), s_emMat(2, plotIdx), 30*neMat(1,plotIdx), 30*neMat(2, plotIdx),...
                'LineWidth',2, 'Color', 'r', 'MaxHeadSize', 4);

axis square
%axis equal
axis([0, 500, -150, 350])
xlabel('x (px)', 'Fontsize', 16)
ylabel('y (px)', 'Fontsize', 16)
grid on
set(h1, 'Position', [0.08, 0.12, 0.4, 0.8])
set(h1, 'FontSize', 16)
set(h1, 'YTick', -150:100:350)

h2 = subplot(1,2,2);
% plot(p_midX, TCI*1e7, 'Color', 'm', 'LineWidth', 2)
% hold on
% plot(p_midX2, TCI2*1e7, '--', 'LineWidth', 2)
xlabel('x (px)', 'Fontsize', 16)
ylabel('TCI', 'Fontsize', 16)
% legend('R = 1/3')
axis([0, 450, 0, 50])
axis square
grid on
set(h2, 'Position', [0.56, 0.12, 0.4, 0.8])
set(h2, 'FontSize', 16)
% hold on

%% animatation
video = VideoWriter('TCIVideo');
myVideo.FrameRate = 10;
open(video)
for i = 1:length(TCI)-1
    line(h2, [p_midX(1,i), p_midX(1,i+1)], [TCI(1,i)*1e7, TCI(1, i+1)*1e7], 'Color', 'm', 'LineWidth', 2);
    hold on
    set(p_line, 'XData', [C(1, i), s_erMat(1, i)], 'YData', [C(2, i), s_erMat(2,i)])
    set(neArrow, 'XData', s_emMat(1,i), 'YData', s_emMat(2,i), 'UData', 30*neMat(1, i), 'VData', 30*neMat(2,i))
    pause(0.1)
    
    frame = getframe(gcf);
    writeVideo(video, frame);
end  
close(video)


%% function from MMU-UK for open uniform B-spline curves
function S = open_quadratic_bspline(P, t)
M1 = 1/2 * [2 -4 2; -3 4 0; 1 0 0];
M2 = 1/2 * [1 -2 1; -2 2 1; 1 0 0];
M3 = 1/2 * [1 -2 1; -3 2 1; 2 0 0];
T = [t.^2; t; t.^0];

% get n
n = size(P, 2);

% calculate the first Bezier curve
B = P(:, 1:3) * M1 * T;
S = B;
% middle curves
for i = 2 : n-3
    B = P(:, i:i+2) * M2 * T;
    S = [S B];
end
% calculate the last Bezier curve
B = P(:, n-2:n) * M3 * T;
S = [S B];

end


%% calculate task-oriented contact index (TCI)
function [s_r, TCI] = calcTCI(C, selIdx, p_length, s)
    % find s_er
    s_l = C(:, selIdx);
    s_r = [];
    thresholdLen = p_length / 24;
    for i = selIdx : 2 : size(C,2)
        s_r = C(:, i);
        if (abs(norm(s_r - s_l) - p_length) <= thresholdLen)
            break
        end
    end
    
    % angle features
    Eindictor = zeros(3,1); % 0-El, 1-E, 2-Er
    distances = zeros(3,1);
    W = zeros(6,1);
    s_0 = s(:, 1);
    s_1 = s(:, 2);
    s_2 = s(:, 3);
    x_0 = s_0(1,1); y_0 = s_0(2,1);
    x_1 = s_1(1,1); y_1 = s_1(2,1);
    x_2 = s_2(1,1); y_2 = s_2(2,1);
    
    v_1 = s_1 - s_0;
    v_2 = s_2 - s_0;
    
    t_0 = v_1' * v_2;
    vn_1 = norm(v_1);
    vn_2 = norm(v_2);
    t_1 = t_0 / vn_1 / vn_2;
    y = acos(t_1);
    Jy = zeros(1, 6);
    
    Jy(1,1) = (-1/sqrt(1 - t_1^2)) * ((2*x_0 - x_1 - x_2) * vn_1^2 * vn_2^2 - t_0 * ...
                ((x_0 - x_1) * vn_2^2 + (x_0 - x_2) * vn_1^2)) / vn_1^3 / vn_2^3;
    Jy(1,2) = (-1/sqrt(1 - t_1^2)) * ((2*y_0 - y_1 - y_2) * vn_1^2 * vn_2^2 - t_0 * ...
                ((y_0 - y_1) * vn_2^2 + (y_0 - y_2) * vn_1^2)) / vn_1^3 / vn_2^3;
    Jy(1,3) = (-1/sqrt(1 - t_1^2)) * ((x_2 - x_0) * vn_1^2 * vn_2 - (x_1 - x_0) * t_0 * vn_2)...
                / vn_1^3 / vn_2^2;
    Jy(1,4) = (-1/sqrt(1 - t_1^2)) * ((y_2 - y_0) * vn_1^2 * vn_2 - (y_1 - y_0) * t_0 * vn_2)...
                / vn_1^3 / vn_2^2;
    Jy(1,5) = (-1/sqrt(1 - t_1^2)) * ((x_1 - x_0) * vn_1 * vn_2^2 - (x_2 - x_0) * t_0 * vn_1)...
                / vn_1^2 / vn_2^3;
    Jy(1,6) = (-1/sqrt(1 - t_1^2)) * ((y_1 - y_0) * vn_1 * vn_2^2 - (y_2 - y_0) * t_0 * vn_1)...
                / vn_1^2 / vn_2^3;
    
    % p plane related parameters
    a = s_r(1,1) - s_l(1,1);
    b = s_r(2,1) - s_l(2,1);
    cl = -a * s_l(1,1) - b * s_l(2,1);
    cr = -a * s_r(1,1) - b * s_r(2,1);
    s_em = (s_l + s_r) / 2;
    refl = a*s_em(1,1) + b*s_em(2,1) + cl;
    refr = a*s_em(1,1) + b*s_em(2,1) + cr;
    divCof = 1 / norm(s_r - s_l);
    
    for i = 1:3
        linel = a * s(1,i) + b * s(2,i) + cl;
        liner = a * s(1,i) + b * s(2,i) + cr;
        if (refl * linel >= 0 && refr * liner >= 0)
            Eindictor(i,1) = 1;
            distances(i,1) = divCof * abs(b * s(1,i) - a * s(2,i) - b * s(1,i) + a * s(2,i));
        elseif(refl * linel < 0)
            Eindictor(i,1) = 0;
            distances(i,1) = divCof * abs(linel);
        else
            Eindictor(i,1) = 2;
            distances(i,1) = divCof * abs(liner);
        end    
    end
    epsilonE = 10;
    epsilonp = 100;
    
    for i = 1:3
        if (Eindictor(i,1) == 1)
           W(2*i-1, 1) = 1 / (distances(i, 1) + epsilonE);
           W(2*i, 1) = W(2*i-1,1);
        else
           W(2*i-1, 1) = 1 / (distances(i, 1) + epsilonp);
           W(2*i, 1) = W(2*i-1, 1);
        end
    end
    M = abs(Jy) * W;
    TCI = norm(M)^2;
end