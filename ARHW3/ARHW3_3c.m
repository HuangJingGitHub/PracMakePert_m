%%% ENGG5402 HW3_3 (b)
function xdot = ARHW3_3c(t, x)
xdot = zeros(2,1);
xdot(1) = x(2);
xdot(2) = -2*sqrt(10)*x(2) - 10*x(1) + 0.1/2;
% xdot(1) = x(2);
% xdot(2) = 2*x(2)^2 - 3*x(1)*x(2);
end