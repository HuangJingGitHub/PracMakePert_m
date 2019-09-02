clear;
%addpath_all();
rosshutdown;
rosinit;
psm_q_initial = [0 0 0.18 0 0 0]';
p2 = psm('PSM2');
p2.move_joint(psm_q_initial);

[pub, jointStateMsg] = rospublisher('/dvrk/PSM1/state_joint_current','sensor_msgs/JointState');
jointStateMsg.Name = {'outer_yaw';'outer_pitch';'outer_insertion';'outer_roll';'outer_wrist_pitch';'outer_wrist_yaw'};
sub = rossubscriber('/dvrk/MTMR/state_joint_current');
pause(0.1);
tele = teleOp(sub.LatestMessage.Position,psm_q_initial,pub,jointStateMsg);
callback = @(src,msg)(tele.callback_update_mtm_q(msg.Position));
sub = rossubscriber('/dvrk/MTMR/state_joint_current',callback,'BufferSize',100);

%tele2 = teleOp();
%tic
%tele2.run(ones(7,1));
%toc
%tic
%tele2.run(zeros(7,1));
%toc
