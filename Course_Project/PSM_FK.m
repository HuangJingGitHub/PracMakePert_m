function T = PSM_FK(psm)
%%% Foreward kinematics of dVRK Patient Side Manipulator(PSM)

T = eye(4);
n = size(psm.DH, 1);

for i = 1:n
    alp = psm.DH(i,2);
    a   = psm.DH(i,3);
    d   = psm.DH(i,4);
    the = psm.DH(i,5);
    Ti = [cos(the)          -sin(the)         0         a;
          sin(the)*cos(alp) cos(the)*cos(alp) -sin(alp) -sin(alp)*d;
          sin(the)*sin(alp) cos(the)*sin(alp) cos(alp)  cos(alp)*d;
          0                 0                 0         1];
    T = T*Ti;
end

end