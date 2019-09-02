clear
close all
clc

tic
load('dvrk_mtm_psm.mat');

%steps = size(mtm_q, 2);
steps = 2000;
mtm_test_q = zeros(7, 1);
psm_test_q = zeros(6, 1);
psm_test_q(1, 1) = 0.25;
psm_test_q(2, 1) = -0.25;
psm_test_q(3, 1) = 0.05;
robot = teleOp_test_trajectory(mtm_q(:,1), psm_q_initial);
%robot = teleOp_test_trajectory(zeros(7, 1), psm_test_q);
error_log = zeros(7, steps);

for i = 1 : steps
%     robot.run(mtm_q(:,i));
    [~, error_log(1:6, i)] = robot.run(mtm_test_q);
    error_log(7, i) = norm(error_log(1:6, i));
end
a = robot.test_0902(:,1:2000);
figure
plot3(a(1,:),a(2,:),a(3,:))
axis square
toc