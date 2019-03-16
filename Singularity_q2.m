clear 
close all
L = 0.0879;
W = 0.0462;
beta = pi/4;
gamma = pi/4;
R2D = 180 / pi;

d = 0:0.005:2;
q_2 = zeros(1, length(d));
for i = 1:length(d)
    q_2(i) = atan((L + d(i) * sin(beta)) / (W + d(i) * cos(beta)) / cos(gamma));
end
q_2 = q_2 * R2D;

figure
plot(d, q_2)