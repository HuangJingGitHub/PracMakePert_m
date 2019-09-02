function [Re, x_dif] = xdif_matrix(T_dsr, T_act)
    %xerr  = eye(4,4);
    x_dsr = T_dsr(1:3, 4);
    x_act = T_act(1:3, 4);
    R_dsr = T_dsr(1:3, 1:3);
    R_act = T_act(1:3, 1:3);
    Re = R_dsr * R_act'; 
    %xerr(1:3, 1:3) = Re;
    %xdif = x_dsr - Re * x_act;
    %xerr(1:3, 4) = xdif;
    x_dif = x_dsr - x_act;
end