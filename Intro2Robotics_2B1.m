clear;
clc;
close all;
%%%Part a)
DR = pi/180;
angs = input('Enter alpha, beta, gamma (deg):\n');
alp = angs(1) * DR;
bet = angs(2) * DR;
gam = angs(3) * DR;

Pba = input('Enter Pba (in array form):\n');
Trpy = rpy2tr(alp, bet, gam);                % Croke's toolbox is used.
Ttrn = transl(Pba);
Tba = Ttrn * Trpy;

%%%Part b)
Pb = input('Enter Pb (column-wise):\n');
Pa = Tba * [Pb; 1];

%%%Part c)
Tba_inv = inv(Tba);
disp('I - Tba_inv*Tba =');
Result = eye(4) - Tba_inv * Tba
disp('I - Tba*Tba_inv =');
Result = eye(4) - Tba * Tba_inv

%%%Part d)
Tcb = transl([3 0 1]) * rpy2tr(0, 20*DR, 0);
Tca = Tba * Tcb;                            % d.i)

Tba = Tca * inv(Tcb);                       % d.ii)
Tcb = inv(Tba) * Tca;                       % d.iii)

