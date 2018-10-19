function x = R3fk(q)
    x(1,1) = cos(q(1)) + cos(q(1)+q(2)) + cos(q(1)+q(2)+q(3));
    x(2,1) = sin(q(1)) + sin(q(1)+q(2)) + sin(q(1)+q(2)+q(3));
end