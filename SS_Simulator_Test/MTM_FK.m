function T = MTM_FK(mtm)
%%% Foreward kinematics of dVRK Master Tool Manipulator(MTM) 

T = eye(4);
frame_num = size(mtm.DH, 1);     % Get number of frames from base frame to tip frame. 

for i = 1 : frame_num
    alp = mtm.DH(i,2);
    a   = mtm.DH(i,3);
    d   = mtm.DH(i,4);
    the = mtm.DH(i,5);
    Ti = [cos(the)           -sin(the)          0          a;
          sin(the)*cos(alp)  cos(the)*cos(alp)  -sin(alp)  -sin(alp)*d;
          sin(the)*sin(alp)  cos(the)*sin(alp)  cos(alp)   cos(alp)*d;
          0                  0                  0          1];
    T = T*Ti;
end

end