clear
close all

DR = pi/180;
%%% T is the transformation matrix from frame{i-1} to frame{i}
%%% and is obtained from to Craig's book page75
syms alp a the d
Rx = [ 1 0        0         0; 
       0 cos(alp) -sin(alp) 0;
       0 sin(alp) cos(alp)  0; 
       0 0        0         1];
Dx = [ 1 0 0 a;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
Rz = [ cos(the) -sin(the) 0 0;
       sin(the) cos(the)  0 0;
       0        0         1 0;
       0        0         0 1];
Dz = [ 1 0 0 0;
       0 1 0 0;
       0 0 1 d;
       0 0 0 1];
T = Rx*Dx*Rz*Dz;     
% T is the result given by equation(3.6) in Craig's book

%%% Get the transfornation matrix from the base frame to the end effector.
syms L1 L2 L3 the1 the2 the3
T0_1 = subs(T, [alp, a, the, d], [0, 0, sym('the1'), 0]);
T1_2 = subs(T, [alp, a, the, d], [0, sym('L1'), sym('the2') 0]);
T2_3 = subs(T, [alp, a, the, d], [0, sym('L2'), sym('the3') 0]);
T3_H = [ 1 0 0 sym('L3');
         0 1 0 0;
         0 0 1 0;
         0 0 0 1];
T0_H = T0_1*T1_2*T2_3*T3_H;


%%% Use Croke's toolbox to establish this 3R robot
%%% Assume theta=[10 20 30]'(deg), L=[1 1 0.5] (m)
L(1) = Link([0, 0, 0, 0]);         
L(2) = Link([10*DR, 0, 1, 0]);     
L(3) = Link([20*DR, 0, 1, 0]);
L(4) = Link([30*DR, 0, 0.5, 0]);
% The attributes for Link() in order is: theta(joint angle), d(offset),
% a(length), alpha(twist). Refer to the Croke's book.

robot3R = SerialLink(L, 'name', '3R_ROT');
q = [0 10*DR 20*DR 30*DR];
T0_H1 = robot3R.fkine(q);
T0_H2 = double(subs(T0_H, [L1, L2, L3, the1, the2, the3],...
                   [1, 1, 0.5, q(2), q(3), q(4)]));
% The transformation matrices given by both function are the same.
robot3R.plot(q);
