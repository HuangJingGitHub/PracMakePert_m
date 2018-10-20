function qdotnull = R3MaxManipty(k0,q)
%%% Maximizing manipulability, caculate the pratial derivate of objective
%%% function of 3R manipulator.
%%% Symbolic computation is very slow when scale rises, so we use
%%% analytical computation though the expression is very complicated.
     s1 = q(1);
     s12 = q(1) + q(2);
     s123 = q(1) + q(2) + q(3);
     J = [-sin(s1)-sin(s12)-sin(s123) -sin(s12)-sin(s123) -sin(s123);
          cos(s1)+cos(s12)+cos(s123) cos(s12)+cos(s123) cos(s123)];
     JH = J*J';
     pwpq1 = (2*J(1,1)*(-J(2,1)) + 2*J(1,2)*(-J(2,2)) + 2*J(1,3)*(-J(2,3)))*JH(2,2)...
             + JH(1,1)*(2*J(2,1)*J(1,1) + 2*J(2,2)*J(1,2) + 2*J(2,3)*J(1,3))...
             - 2*JH(1,2)*(-J(2,1)^2+J(1,1)^2 - J(2,2)^2+J(1,2)^2 -J(2,3)^2+J(1,3)^2);
     pwpq2 = (2*J(1,1)*(-J(2,2)) + 2*J(1,2)*(-J(2,2)) + 2*J(1,3)*(-J(2,3)))*JH(2,2)...
             + JH(1,1)*(2*J(2,1)*J(1,2) + 2*J(2,2)*J(1,2) + 2*J(2,3)*J(1,3))...
             - 2*JH(1,2)*(-J(2,2)*J(2,1)+J(1,1)*J(1,2) - J(2,2)^2+J(1,2)^2 - J(2,3)^2+J(1,3)^2);
     pwpq3 = (2*J(1,1)*(-J(2,3)) + 2*J(1,2)*(-J(2,3)) + 2*J(1,3)*(-J(2,3)))*JH(2,2)...
             + JH(1,1)*(2*J(2,1)*J(1,3) + 2*J(2,2)*J(1,3) + 2*J(2,3)*J(1,3))...
             - 2*JH(1,2)*(-J(2,3)*J(2,1)+J(1,1)*J(1,3) - J(2,3)*J(2,2)+J(1,2)*J(1,3) - J(2,3)^2+J(1,3)^2);
     qdotnull = k0/(-2*sqrt(det(JH)))*[pwpq1; pwpq2; pwpq3];
end