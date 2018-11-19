function psm = dVRK_PSMmodel(q)

l_RCC          = 0.4318;
l_tool         = 0.4162;
l_Pitch2Yaw    = 0.0091;
l_Yaw2CtrlPnt  = 0.0102; 

psm.DH = [
    % type |  alpha  |   a   |         d         |       theta
       1      pi/2       0             0               pi/2 + q(1);
       1      -pi/2      0             0               q(2) - pi/2;
       2      pi/2       0             -l_RCC + q(3)   0;
       1      0          0             l_tool          pi/2 + q(4);
       1      -pi/2      0             0               q(5) - pi/2;
       1      -pi/2      l_Pitch2Yaw   0               q(6) - pi/2;
       1      -pi/2      0             l_Yaw2CtrlPnt   pi/2       ];   % pi/2 + q(7), q(7) is fixed as 0.

end