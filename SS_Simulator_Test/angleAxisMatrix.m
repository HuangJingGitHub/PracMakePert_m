function Rr = angleAxisMatrix(axis, gamma, position)
    Rr = eye(4);
    T1 = eye(4);
    T2 = eye(4);
    T1(1:3, 4) = -position;
    T2(1:3, 4) = position;
    x = axis(1) / norm(axis);
    y = axis(2) / norm(axis);
    
    gamma = gamma / pi * 180;
    alpha = asin(y / sqrt(x^2 + y^2)) / pi * 180;
    beta = asin(sqrt(x^2 + y^2)) / pi * 180;
    
    Rr(1:3, 1:3) =  rotz(alpha) * roty(beta) * rotz(gamma) * roty(-beta) * rotz(-alpha);
    Rr = T2 * Rr * T1;
end
    
    
    
    
    
