function psm_v = PSM_Vel_Cal(T_cur, T_pre)
%%% To calculate the tip velocity of PSM according to 2 homogenerous
%%% matrices given at current and previous instants.

fre = 1000;
t_sample = 1 / fre;

R_cur = T_cur(1:3, 1:3);
R_pre = T_pre(1:3, 1:3);

R_pre2cur = R_cur / R_pre;
theta = acos((trace(R_pre2cur) - 1) / 2);
K = [ R_pre2cur(3, 2) - R_pre2cur(2, 3);
      R_pre2cur(1, 3) - R_pre2cur(3, 1);
      R_pre2cur(2, 1) - R_pre2cur(1, 2) ];

epsilon = 1e-4;
if theta > epsilon && theta < pi - epsilon          %%% To avoid singular scenarios.
    K = theta * K / (2*sin(theta));
else
    K = theta * K / 1e-3;
end

omega = K / t_sample;
xdot = (T_cur(1:3, 4) - T_pre(1:3, 4)) / t_sample;
psm_v = [xdot; omega];

end