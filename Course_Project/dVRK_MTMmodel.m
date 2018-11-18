function mtm = dVRK_MTMmodel(q)

l_arm      = 0.2794;
l_forearm1 = 0.3048;
l_forearm2 = 0.0597;
l_forearm  = l_forearm1 + l_forearm2;
h          = 0.1506;

% mtm.DH = [
%     % type |  alpha  |   a   |      d      |   theta
%        1      pi/2       0          0          0;
%        1      0          l_arm      0          0;
%        1      -pi/2      l_forearm  0          0;
%        1      pi/2       0          h          0;
%        1      -pi/2      0          0          0;
%        1      pi/2       0          0          0;
%        1      0          0          0          0 ];

mtm.DH = [
    % type |  alpha  |   a   |      d      |   theta
       1      0          0          0          -pi/2 + q(1);
       1      pi/2       0          0          -pi/2 + q(2);
       1      0          l_arm      0          pi/2 + q(3);
       1      -pi/2      l_forearm  h          q(4);
       1      pi/2       0          0          q(5);
       1      -pi/2      0          0          -pi/2 + q(6);
       1      pi/2       0          0          pi/2 + q(7) ];

   
% for i = 1:length(q)
%     if mtm.DH(i, 1) == 1
%         mtm.DH(i, 5) = q(i);
%     else
%         mtm.DH(i, 4) = q(i);
%     end
% end

end