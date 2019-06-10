function psm = dVRK_PSMmodel_SSWND(q)
%%% Single-Site Wristed Needle Driver %%%
l_56  = 0.00466; % 0.004;
l_67  = 0.019;   % 0.012; 
SS.bend     = pi/4;
SS.twist    = pi/6; 
SS.W        = 0.039; %0.0462;
SS.L        = 0.094; %0.0879;
SS.offset   = SS.twist - pi/2;
SS.d_0      = -0.041; %-0.05;
SS_pars = SS_Parameters_Cal(SS);

psm.DH = [
    % type  |  alpha  |        a         |         d         |       theta
       1      pi/2            0             0               pi/2 + q(1);
       1      -pi/2           0             SS_pars.d_12    q(2) + SS_pars.the_12;
       2      SS_pars.alp_23  SS_pars.a_23  q(3)            SS_pars.the_23;
       1      0               0             SS_pars.d_34    q(4) + SS.offset - pi/4;   % theta: SS.offset
       1      -pi/2           0             0               q(5) - pi/2;
       1      -pi/2           l_56          0               q(6) - pi/2;
       1      -pi/2           0             l_67            pi/2       ];  

end

function SS_pars = SS_Parameters_Cal(SS)
W = SS.W;
L = SS.L;
d_0 = SS.d_0;
beta = SS.bend;
gamma = SS.twist;
cst1 = W/tan(beta);
cst2 = sqrt((W*cos(gamma))^2 + cst1^2);

d_12 = (L - cst1) * W^2 * sin(gamma) / (tan(beta) * cst2^2);
the_12 = pi/2 + acos(cst1 / cst2);
alp_23 = 3*pi/2 - atan(W * sin(gamma)/cst2);
a_23 = (L - cst1) * W * cos(gamma) / cst2;
the_23 = -pi/2 - asin(cst1 / cst2 * sin(gamma));
d_34 = (L - cst1) * cst1 / cst2 / (cst2 * sin(beta) / W) + W / sin(beta) + d_0;

SS_pars.d_12 = d_12;
SS_pars.the_12 = the_12;
SS_pars.alp_23 = alp_23;
SS_pars.a_23 = a_23;
SS_pars.the_23 = the_23;
SS_pars.d_34 = d_34;

end