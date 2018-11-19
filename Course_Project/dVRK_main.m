%%% ENGG5402 Advanced Robotics Course Project: dVRK_Simulator
%%% 
clear
close all
clc

load('dvrk_mtm_psm.mat');
fre = 1000;
t_sample = 1 / fre;

num_sample = size(mtm_q, 2);
mtm_x_test = zeros(4,4,num_sample);
psm_x_dsr_test = zeros(4,4,num_sample);
psm_xdot_dsr_test = zeros(6, num_sample); 
T_psm_mtm =  psm_x_dsr(:,:,1) / mtm_x(:,:,1) ;  % Transformation matrix from mtm base frame to psm base frame.

%%% Finish the 1st step calculation, for in the loop structure dependency on
%%% precious result exists.
q_mtm = mtm_q(:, 1);
mtm = dVRK_MTMmodel(q_mtm);
mtm_x_test(:,:,1) = MTM_FK(mtm);
psm_x_dsr_test(:,:,1) =  T_psm_mtm * mtm_x_test(:,:,1);
    
T1 = zeros(1,num_sample - 1);
T2 = zeros(1, num_sample -1);
for i = 2 : num_sample
    q_mtm = mtm_q(:, i);
    mtm = dVRK_MTMmodel(q_mtm);
    mtm_x_test(:,:,i) = MTM_FK(mtm);
    psm_x_dsr_test(:,:,i) =  T_psm_mtm * mtm_x_test(:,:,i);
    psm_xdot_dsr_test(:, i-1) = PSM_Vel_Cal(psm_x_dsr_test(:,:,i), psm_x_dsr_test(:,:,i-1));
    T1(1,i-1) = norm(psm_xdot_dsr_test(:, i-1));
    T2(1,i-1) = norm(psm_xdot_dsr(:, i-1));
end  

psm_q_test = zeros(6, num_sample);
psm_x_act_test = zeros(4, 4, num_sample);
psm_q_test(:, 1) = psm_q_initial;
lambda = diag([100 100 100 20 20 20]);
%xer = zeros(6, num_sample);
ang_er = zeros(1, num_sample);

for i = 1 : num_sample
    q_psm = psm_q_test(:,i);
    psm = dVRK_PSMmodel(q_psm);
    [psm_x_act_test(:,:,i), J] = PSM_FK(psm);
    %diff(:,:,i) = psm_x_act_test(:,:,i) - psm_x_dsr_test(:,:,i);
    xer = tr2vec(psm_x_dsr_test(:,:,i)) - tr2vec(psm_x_act_test(:,:,i));
    ang_er(1, i) = norm(xer(1:3));
    qdot_psm = J \ (psm_xdot_dsr_test(:,i) + lambda * xer);
    psm_q_test(:, i+1) = psm_q_test(:, i) + qdot_psm * t_sample;    
end

figure
plot(ang_er);

figure
% plot(T1);
% hold on
plot(T2 - T1);


i = 1;
     q_psm = zeros(6, 1);
     q_psm(6,1) = pi/2;
     psm = dVRK_PSMmodel(q_psm);
     [psm_x_test(:,:,i), J] = PSM_FK(psm)

% T_psm_mtm1 = psm_x_ref / mtm_x_ref;          
