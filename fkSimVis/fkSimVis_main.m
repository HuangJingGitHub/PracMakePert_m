%%% Forward kinematics and visualization for manipulator.
%%% HUANG Jing @ CUHK MAE

clear
clc
close all

syms L q1 q2 q3 q4 q5 q6;           % Define the linkage length and joint angles needed.
D2R = pi/180;                       % Degree to radian constant.

%%% DH table example: PRRRRP robot
DH(1,:) = [ 0     0   q1         pi/2];
DH(2,:) = [pi/2   0   0          q2  ];
DH(3,:) = [-pi/2  L   0          q3  ];
DH(4,:) = [pi/2   0   0          q4+pi/2];
DH(5,:) = [pi/4   0   sqrt(2)*L  q5  ];
DH(6,:) = [pi/4   0   L+q6       pi/2];
DH(7,:) = [pi/2   0   0          0   ];

%%% Transformation from frame{0} to the end-effector.
T1 = eye(4);
T3 = eye(4);
T5 = eye(4);
T7 = eye(4);
for i=1:7
    alp = DH(i,1);
    a   = DH(i,2);
    d   = DH(i,3);
    the = DH(i,4);
    Ti = [cos(the)          -sin(the)         0         a;
          sin(the)*cos(alp) cos(the)*cos(alp) -sin(alp) -sin(alp)*d;
          sin(the)*sin(alp) cos(the)*sin(alp) cos(alp)  cos(alp)*d;
          0                 0                 0         1];
    T7 = T7*Ti;
    if i == 1
        T1 = T7;
    end
    
    if i == 3
        T3 = T7;
    end
    
    if i == 5 
        T5 = T7;
    end
end
T1 = subs(T1(1:3,4),[L q3 q4],[1 0 0]);  % For the joint, just the Cartesian coordinates are needed, 
T4 = -L*T3(1:3,2)+T3(1:3,4);             % the rotation part won't be used.Set q3,q4 as 0 in this example.
T3 = subs(T3(1:3,4),[L q3 q4],[1 0 0]);  % Actually, the 4th column of transformation matrix just represents
T4 = subs(T4,[L q3 q4],[1 0 0]);         % the position of the frame, not necessaryly is the actual position
T5 = subs(T5(1:3,4),[L q3 q4],[1 0 0]);  % of the joint. That's why the T4 is obtained differently.
T7 = subs(T7,[L q3 q4],[1 0 0 ]);        % For tip, the oritation part need to be keeped.

%%% Simulation configurations
dt = 0.02;
Time  = 5;
t = 0:dt:Time;
steps = length(t);
Q1 = 0.1 * sin(2*pi*t);
Q2 = pi / 6 * sin(2*pi*t);
Q3 = zeros(1,steps);
Q4 = zeros(1,steps);
Q5 = Q2;
Q6 = Q1;

X_1 = zeros(3,steps);     % X_1 stores Cartesian coordinate of the second last joint(joint5).
X_3 = zeros(3,steps); 
X_4 = zeros(3,steps); 
X_5 = zeros(3,steps); 
X   = zeros(3,steps);     % X stores Cartesian coordinate of the end-effector/tip.
Ori = zeros(3,3*steps);   % Ori stores the ratation matrix of the end-effector.

for i = 1:steps           % Traverse all the time point.
    Tt = subs(T7,[q1 q2 q5 q6],[Q1(i) Q2(i) Q5(i) Q6(i)]);
    Tt = double(Tt);
    X(:,i) = Tt(1:3,4);
    X_1(:,i)= double(subs(T1,[q1 q2 q5 q6],[Q1(i) Q2(i) Q5(i) Q6(i)]));
    X_3(:,i)= double(subs(T3,[q1 q2 q5 q6],[Q1(i) Q2(i) Q5(i) Q6(i)]));
    X_4(:,i)= double(subs(T4,[q1 q2 q5 q6],[Q1(i) Q2(i) Q5(i) Q6(i)]));
    X_5(:,i)= double(subs(T5,[q1 q2 q5 q6],[Q1(i) Q2(i) Q5(i) Q6(i)]));
    Ori(:,3*i-2:3*i) = Tt(1:3,1:3);
end

fkVis(steps,dt,Ori,X,X_1,X_3,X_4,X_5);
 
%%%******Plot the all joints******%%%
%%% It's noticed that if this figure part is placed before the tip
%%% trajectory plot part. Then error will be caused in invocation 
%%% of arrow3. This may be caused by the unknown figure handle conflict.
%%%
 figure('Position',[0 -200 1050 800]);
 H1 = subplot(3,2,1);
 plot(t, Q1,'k','linewidth',2);
 xlabel('t \it{(s)}');
 ylabel('q_1 \it{(m)}');
 
 H2 = subplot(3,2,2);
 plot(t, Q2/D2R,'k','linewidth',2);
 xlabel('t \it{(s)}');
 ylabel('q_2 \it{(deg)}');
 
 H3 = subplot(3,2,3);
 plot(t, Q3/D2R,'k','linewidth',2);
 xlabel('t \it{(s)}');
 ylabel('q_3 \it{(deg)}');
 
 H4 = subplot(3,2,4);
 plot(t, Q4/D2R,'k','linewidth',2)
 xlabel('t \it{(s)}');
 ylabel('q_4 \it{(deg)}');
 
 H5 = subplot(3,2,5);
 plot(t, Q5/D2R,'k','linewidth',2)
 xlabel('t \it{(s)}');
 ylabel('q_5 \it{(deg)}');
 
 H6 = subplot(3,2,6);
 plot(t, Q6,'k','linewidth',2)
 xlabel('t \it{(s)}');
 ylabel('q_6 \it{(m)}');
