function xdot = ARHW4_2ii(t, x)
%%% State variables are chosen as q1, q1dot, q2, q2dot in order.
xdot = zeros(4,1);

tau = [0; 0];
s2 = sin(x(3));
c2 = cos(x(3));
qdot_1 = x(2);
qdot_2 = x(4);
qdot = [x(2); x(4)];
M = [10 + s2^2    c2;
         c2       2 ];
C = [s2*c2*qdot_2   s2*c2*qdot_1 - s2*qdot_2;
     -s2*c2*qdot_1  0 ];
g = [0; 10*s2];
qddot = M \ (tau - C*qdot - g);

xdot(1) = x(2);
xdot(3) = x(4);
xdot(2) = qddot(1);
xdot(4) = qddot(2);
end

