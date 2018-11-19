function [T, J] = PSM_FK(psm)
%%% Foreward kinematics of dVRK Patient Side Manipulator(PSM)

T = eye(4);
frame_num = size(psm.DH, 1);     % Get number of frames from base frame to tip frame.
J = zeros(6, frame_num - 1);     % Jacobian of the MTM.
Z = zeros(3, frame_num);         % To store the Z unit vector coordinates of each transformation matrix.
P = zeros(3, frame_num);         % To store the origin position of each transformation matrix.

for i = 1 : frame_num
    alp = psm.DH(i,2);
    a   = psm.DH(i,3);
    d   = psm.DH(i,4);
    the = psm.DH(i,5);
    Ti = [cos(the)            -sin(the)           0           a;
          sin(the)*cos(alp)   cos(the)*cos(alp)   -sin(alp)   -sin(alp)*d;
          sin(the)*sin(alp)   cos(the)*sin(alp)   cos(alp)    cos(alp)*d;
          0                   0                   0           1];
    T = T*Ti;
    Z(:, i) = T(1:3, 3);
    P(:, i) = T(1:3, 4);
end

for i = 1 : frame_num - 1
    type = psm.DH(i,1);
    if type == 1
        J(1:3, i) = cross(Z(:,i), P(:,end) - P(:,i));
        J(4:6, i) = Z(:,i);
    else
        J(1:3, i) = Z(:,i);
    end
end

end