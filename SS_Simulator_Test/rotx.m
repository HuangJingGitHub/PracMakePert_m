function R = rotx(ang)
    ang = ang / 180 * pi;
    R = [1          0              0;
         0          cos(ang)       -sin(ang);
         0          sin(ang)       cos(ang)];
end