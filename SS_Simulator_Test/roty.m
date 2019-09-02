function R = roty(ang)
    ang = ang / 180 * pi;
    R = [cos(ang)       0       sin(ang);
         0              1       0;
         -sin(ang)      0       cos(ang)];
end