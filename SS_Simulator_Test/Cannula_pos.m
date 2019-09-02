function T = Cannula_pos(q)
    SSmodel = SingSite_model(q);
    T = SS_fardk(SSmodel);
    T = T(:,end);
end

function ss = SingSite_model(q)
ss.bend     = pi/4;
ss.twist    = pi/6;
ss.L        = 0.0879;
ss.W        = 0.0462;
ss.DH = [
    % type |  alpha  |   a   |         d         |       theta
       1      pi/2       0              0                   pi/2 + q(1);
       1      -pi/2      0             0                    q(2) - pi/2;
       3      pi/2       0      ss.L - ss.W/tan(ss.bend)    3*pi/2 - ss.twist; 
       2      -ss.bend   0      ss.W/sin(ss.bend)           pi    ]; 

end

function T = SS_fardk(ss)

T = eye(4);
frame_num = size(ss.DH, 1);      % Get number of frames from base frame to tip frame.
P = zeros(3, frame_num);         % To store the origin position of each transformation matrix.

for i = 1 : frame_num
    alp = ss.DH(i,2);
    a   = ss.DH(i,3);
    d   = ss.DH(i,4);
    the = ss.DH(i,5);
    Ti = [cos(the)            -sin(the)           0           a;
          sin(the)*cos(alp)   cos(the)*cos(alp)   -sin(alp)   -sin(alp)*d;
          sin(the)*sin(alp)   cos(the)*sin(alp)   cos(alp)    cos(alp)*d;
          0                   0                   0           1];
    T = T*Ti;
    P(:, i) = T(1:3, 4);
end

end