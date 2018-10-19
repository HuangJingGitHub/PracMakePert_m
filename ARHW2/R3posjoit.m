function PJ = R3posjoit(q)
% Position of the joint2 and joint3
    s1 = q(1);
    s12 = q(1) + q(2);
    PJ = zeros(4,1);
    PJ(1) = cos(s1);
    PJ(2) = sin(s1);
    PJ(3) = cos(s1) + cos(s12);
    PJ(4) = sin(s1) + sin(s12);
end