%%% Forward-dynamics solution fot the plannar 3-DOF, 3R robot for motion
%%% over time. Using Croke's toolbox directly. Generally, forward-dynamics
%%% solution is based on solution of dynamic equation for acceleration,
%%% where numerical integration is then used (Integrated forward dynamics). The dynamic equation for
%%% manipulator cannot be transformed to linear state equation.
clear 
close all
clc

%%% Robot configurations
DR = pi/180;
L1 = 4;
L2 = 3;
L3 = 2;

%%% DH parameters
% link twist | link length | link offset | joint angle
alp(1) = 0;  a(1) = L1;  d(1) = 0;  th(1) = 0;
alp(2) = 0;  a(2) = L2;  d(2) = 0;  th(2) = 0; 
alp(3) = 0;  a(3) = L3;  d(3) = 0;  th(3) = 0;

L(1) = Link([th(1),d(1),a(1),alp(1)]);
L(1).mdh = 1;  L(1).m = 20;  L(1).r = [L1/2 0 0];
L(1).I = zeros(3,3);
L(1).I(3,3) = 0.5;

L(2) = Link([th(2),d(2),a(2),alp(2)]);
L(2).mdh = 1;  L(2).m = 15;  L(2).r = [L2/2 0 0];
L(2).I = zeros(3,3);
L(2).I(3,3) = 0.2;

L(3) = Link([th(3),d(3),a(3),alp(3)]);
L(3).mdh = 1;  L(3).m = 10;  L(3).r = [L3/2 0 0];
L(3).I = zeros(3,3);
L(3).I(3,3) = 0.1;

robot3R = SerialLink(L, 'name', '3R_ROT');

%%% Simulation configurations
Time = 5;
dt = 0.1;
t = 0:dt:Time;
q0 = [-60 90 30]*DR;
qd0 = [0 0 0];
%robot3R.plot(q0);
% [ti, q, qd] = robot3R.fdyn(Time, @torqfun, q0, qd0); Something wrong with
% function fdyn.

