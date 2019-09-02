function xerr = xdif_lite(T_dsr, T_act)

    x_dsr = T_dsr(1:3, 4);
    x_act = T_act(1:3, 4);
    R_dsr = T_dsr(1:3, 1:3);
    R_act = T_act(1:3, 1:3);
    Re = R_dsr * R_act'; 
    K =  [Re(3,2) - Re(2,3);
          Re(1,3) - Re(3,1);
          Re(2,1) - Re(1,2) ];
      
    xdif = x_dsr - x_act;
    xerr = [xdif; K ];
end