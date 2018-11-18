%%% ENGG5402 Advanced Robotics Course Project: dVRK_Simulator
%%% 
clear
close all
clc

load('dvrk_mtm_psm.mat');

num_sample = size(mtm_q, 2);
mtm_x_test = zeros(4,4,num_sample);

for i = 1:num_sample
    q_mtm = mtm_q(:, i);
    mtm = dVRK_MTMmodel(q_mtm);
    mtm_x_test(:,:,i) = MTM_FK(mtm);
end  

% i = 1;
%      q_psm = zeros(6, 1);
%      psm = dVRK_PSMmodel(q_psm);
%      psm_x_test(:,:,i) = PSM_FK(psm)

