%%% ENGG5402 HW4_2d, Dynamic manipulability ellipsoid.
%%% The ODE is: M(q, qdot)*qddot + C(q, qdot)*qdot + g(q) = tau

clear
close all
clc

q  = [pi/2 ; pi/2];
s1 = sin(q(1));
s2 = sin(q(2));
c1 = cos(q(1));
c2 = cos(q(2));

M  = [10 + s2^2    c2;
      c2           2 ];
J = [-s1-c1*s2   -s1*c2;
     c1-s1*s2    c1*c2;
     0           s2
     0           c1;
     0           s1;
     1           0 ];
g = [0; 10*sin(q(2))];
%  [U, S, V] = svd(J);
%  Q1 = V/(S'*S)*S'*U';
Q = pinv(J)' * M' * M * pinv(J);
Q = Q(1:3, 1:3);
[U, S, V] = svd(Q);
 
t = 0:0.01:2*pi;
z = S(1,1) * cos(t);
x = S(2,2) * sin(t);
 
x = -1 + x;
z = z - 10;
figure
plot(x,z, 'Linewidth', 2);
hold on
plot(x, z + 10, 'Color', 'red', 'Linewidth', 2);
%axis equal
axis([-4.5, 4.5, -40, 40])
xlabel('X', 'Fontname', 'Times', 'Fontsize', 12);
ylabel('Z', 'Fontname', 'Times', 'Fontsize', 12);
title('Dynamic Manipuliability Ellipsoid', 'Fontname', 'Times', 'Fontsize', 12);
legend('With gravity','g = 0')
 
