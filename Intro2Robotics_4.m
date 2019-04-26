%%% Algebraic solution for the inverse kinematics of a 3R planar
%%% manipulator. (Craig's book chapter4)
%%% Given a transformation matrix from base frame to wrist
%%% frame Tb_w = [cos(phi) -sin(phi) 0 x;
%%%               sin(phi) cos(phi)  0 y;
%%%               0        0         1 0;
%%%               0        0         0 1]
%%% to solve the joint space vector [theta1 theta2 theta3]' generating 
%%% this transformation matrix. See algorithm in John Craig's book P117-120 (4th edition)
%%% L1=4m, L2=3m, L1=2m.
%%% HUANG Jing @CUHK MAE, August 2018.
clear
close all
clc

L1 = 4;
L2 = 3;
L3 = 2;  % Actually, L3 is not used. Because this problem just gives information of
         % the frame{n}(the wrist frame), not the tool frame{T}.
Tb_w = input('Please enter the transformation matrix from base frame to wrist frame:\n');
c_phi = Tb_w(1,1);
s_phi = Tb_w(2,1);
x = Tb_w(1,4);
y = Tb_w(2,4);

c_the2 = (x^2 + y^2 - L1^2 - L2^2) /(2 * L1* L2);
if ~(c_the2 >= -1 && c_the2 <=1)
    disp('Not solvabele transformation matrix is given.');
    return;
end

s_the2 = [ sqrt(1-c_the2^2) -sqrt(1-c_the2^2)];
the2 = [Atan2(s_the2(1), c_the2, 'rad'), Atan2(s_the2(2), c_the2, 'rad')];
k1 = L1 + L2 * cos(the2);
k2 = L2 * sin(the2);
the1(1) = Atan2(y, x, 'rad') - Atan2(k2(1), k1(1), 'rad');
the1(2) = Atan2(y, x, 'rad') - Atan2(k2(2), k1(2), 'rad');
phi = Atan2(s_phi, c_phi, 'rad');
the3 = phi - the1 -the2;

q1 = [the1(1) the2(1) the3(1)]';
q2 = [the1(2) the2(2) the3(2)]';
disp('Solution1 is q1 =');
disp(q1);
disp('Solution2 is q2 =');
disp(q2);

%%% Circular check using Croke's toolbox. Build a manipulator model and
%%% show that the given solution can give the original transformation
%%% matrix.
L(1) = Link([0, 0, 0, 0]);   % Start from link0(base).      
L(2) = Link([0, 0, L1, 0]);     
L(3) = Link([0, 0, L2, 0]);
L(4) = Link([0, 0, 0, 0]);   % Pay attention that the length of link4 is 0 
                             % instead of L3. The frame{W} is affixed at
                             % the end of link3 and can rotate.
robot3R = SerialLink(L, 'name', '3R_PlanarRobot');
checkTb_w1 = robot3R.fkine([0; q1]);
robot3R.plot([0 q1']);
checkTb_w2 = robot3R.fkine([0; q2]);
% checkTb_w1 and checkTb_w2 are coincident with Tb_w entered.
