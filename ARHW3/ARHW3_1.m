%%% ENGG5402 HW3_1, Trajectory generation. Unit for angle: deg

%%% 1 a) Third-order polynomial.
clear
close all;

the    = [15 75];       % Start and end positon
% the_d  = [0 0];         % Start and end anglular velocity
tf     = 3;
dt     = 1/50;
t1      = 0:dt:tf;

% coefficients matrix for angle. Refer to chapter 7 of Craig's book.
a1(1) = the(1);
a1(2) = 0; 
a1(3) = 3*(the(2) - the(1)) / (tf^2);
a1(4) = -2*(the(2) - the(1)) / (tf^3);
% coefficients matrix for angular velocity and acceleration
av1 = [ a1(2) 2*a1(3) 3*a1(4) ];
aa1 = [ 2*a1(3) 6*a1(4)];

%%% Vectorization programming is much preferred than loop. 
T1(1,:) = ones(1, size(t1,2));
T1(2,:) = t1;
T1(3,:) = t1.^2;
T1(4,:) = t1.^3;

q1    = a1*T1(1:4,:);     % angle
q_d1  = av1*T1(1:3,:);    % angular velocity    
q_dd1 = aa1*T1(1:2,:);    % angular acceleration


%%% 1 b) Change of congigurations.
%%% similar procedure
the = [10 100];
% the_d = [0 0];
tf     = 1;
dt     = 1/50;
t2      = 0:dt:tf;

a2(1) = the(1);
a2(2) = 0; 
a2(3) = 3*(the(2) - the(1)) / (tf^2);
a2(4) = -2*(the(2) - the(1)) / (tf^3);
av2 = [ a2(2) 2*a2(3) 3*a2(4) ];
aa2 = [ 2*a2(3) 6*a2(4)];

T2(1,:) = ones(1, size(t2,2));
T2(2,:) = t2;
T2(3,:) = t2.^2;
T2(4,:) = t2.^3;

q2    = a2*T2(1:4,:);     
q_d2  = av2*T2(1:3,:);      
q_dd2 = aa2*T2(1:2,:);    

%%% Plot part. figure1
figure('position', [100 50 850 600]);
H11 = subplot(3,1,1);
plot(t1, q1, 'linewidth', 2)
ylabel('$\theta$','interpreter','LaTex','fontsize', 12);
set(gca,'XTick',[]); 
title('Cubic Polynomial Trajectory','fontname','Times','fontsize', 12);
H12 = subplot(3,1,2);
plot(t1, q_d1, 'linewidth', 2)
ylabel('$\dot{\theta}$','interpreter','LaTex','fontsize', 12);
set(gca,'XTick',[]); 
H13 = subplot(3,1,3);
plot(t1, q_dd1, 'linewidth', 2);
ylabel('$\ddot{\theta}$','interpreter','LaTex','fontsize',12);
xlabel('\it{Time (s)}','fontname','Times','fontsize',12);

%%% figure2
figure('position',[100 50 850 600]);
H21 = subplot(3,1,1);
plot(t1, q1, 'linewidth', 2)
hold on
plot(t2, q2, 'color', 'r', 'linewidth', 2)
ylabel('$\theta$','interpreter','LaTex','fontsize', 12);
set(gca,'XTick',[]);
title('Cubic Polynomial Trajectory','fontname','Times','fontsize', 12);
legd1 = legend('1 a)', '1 b)');
set(legd1,'box','off');

H22 = subplot(3,1,2);
plot(t1, q_d1, 'linewidth',2)
hold on
plot(t2, q_d2, 'color', 'r', 'linewidth', 2)
ylabel('$\dot{\theta}$','interpreter','LaTex','fontsize',12);
set(gca,'XTick',[]); 
legd2 = legend('1 a)', '1 b)');
set(legd2,'box','off');

H23 = subplot(3,1,3);
plot(t1, q_dd1, 'linewidth',2)
hold on
plot(t2, q_dd2, 'color', 'r', 'linewidth', 2)
ylabel('$\ddot{\theta}$','interpreter','LaTex','fontsize',12);
xlabel('\it{Time (s)}','fontname','Times','fontsize',12);
legd3 = legend('1 a)', '1 b)');
set(legd3,'box','off');

%%% 1 c) Linear function with parabolic blends
the    = [15 75];       
tf     = 3;
dt     = 1/100;
t3      = 0:dt:tf;
steps = length(t3);

the_dd1 = 30;
the_dd2 = 100;
tb1 = tf/2 - sqrt((the_dd1*tf)^2 - 4*the_dd1*(the(2) - the(1)))/(2*the_dd1);
tb2 = tf/2 - sqrt((the_dd2*tf)^2 - 4*the_dd2*(the(2) - the(1)))/(2*the_dd2);
n1 = ceil(tb1/dt);
n2 = ceil(tb2/dt);

vb1 = the_dd1*tb1;
q3_1 = zeros(1, steps);
q3_1(1:n1) = the(1) + the_dd1 * t3(1:n1).^2 / 2;
q3_1(n1+1:steps-n1) = the(1) + vb1*tb1/2 + vb1*(t3(n1+1:steps-n1) - tb1);
q3_1(steps-n1+1:steps) = the(1) + vb1*tb1/2 + vb1*(tf-2*tb1) + vb1*(t3(steps-n1+1:steps) - (tf-tb1))...
                         - the_dd1*(t3(steps-n1+1:steps) - (tf-tb1)).^2/2;
                     
q_d3_1 = zeros(1, steps);
q_d3_1(1:n1) = the_dd1 * t3(1:n1);
q_d3_1(n1+1:steps-n1) = vb1;
q_d3_1(steps-n1+1:steps) = vb1 - the_dd1*(t3(steps-n1+1:steps)-(tf-tb1));

q_dd3_1 = zeros(1, steps);
q_dd3_1(1:n1) = the_dd1;
q_dd3_1(steps-n1+1:steps) = -the_dd1;


vb2 = the_dd2*tb2;
q3_2 = zeros(1, steps);
q3_2(1:n2) = the(1) + the_dd2 * t3(1:n2).^2 / 2;
q3_2(n2+1:steps-n2) = the(1) + vb2*tb2/2 + vb2*(t3(n2+1:steps-n2) - tb2);
q3_2(steps-n2+1:steps) = the(1) + vb2*tb2/2 + vb2*(tf-2*tb2) + vb2*(t3(steps-n2+1:steps) - (tf-tb2))...
                         - the_dd2*(t3(steps-n2+1:steps) - (tf-tb2)).^2/2;
q_d3_2 = zeros(1, steps);
q_d3_2(1:n2) = the_dd2 * t3(1:n2);
q_d3_2(n2+1:steps-n2) = vb2;
q_d3_2(steps-n2+1:steps) = vb2 - the_dd2*(t3(steps-n2+1:steps)-(tf-tb2));

q_dd3_2 = zeros(1, steps);
q_dd3_2(1:n1) = the_dd2;
q_dd3_2(steps-n1+1:steps) = -the_dd2;

%%% figure3
figure('position',[100 50 850 600]);
H31 = subplot(3,1,1);
plot(t1, q1, 'linewidth', 2)
hold on
plot(t3, q3_1, 'color', 'r', 'linewidth', 2)
ylabel('$\theta$','interpreter','LaTex','fontsize', 12);
set(gca,'XTick',[]);
title('$Linear\;Function\;with\;Parabolic\;Blends\;(\ddot{\theta}=30\;deg/s^2)$','interpreter','LaTex','fontsize', 12);
legd1 = legend('Cubic Ploynomial', 'Linear&Parabolic');
set(legd1,'box','off');

H22 = subplot(3,1,2);
plot(t1, q_d1, 'linewidth',2)
hold on
plot(t3, q_d3_1, 'color', 'r', 'linewidth', 2)
ylabel('$\dot{\theta}$','interpreter','LaTex','fontsize',12);
set(gca,'XTick',[]); 
legd2 = legend('Cubic Ploynomial', 'Linear&Parabolic');
set(legd2,'box','off');

H23 = subplot(3,1,3);
plot(t1, q_dd1, 'linewidth',2)
hold on
plot(t3, q_dd3_1, 'color', 'r', 'linewidth', 2)
ylabel('$\ddot{\theta}$','interpreter','LaTex','fontsize',12);
xlabel('\it{Time (s)}','fontname','Times','fontsize',12);
legd3 = legend('Cubic Ploynomial', 'Linear&Parabolic');
set(legd3,'box','off');

%%% figure4
figure('position',[100 50 850 600]);
H31 = subplot(3,1,1);
plot(t1, q1, 'linewidth', 2)
hold on
plot(t3, q3_2, 'color', 'r', 'linewidth', 2)
ylabel('$\theta$','interpreter','LaTex','fontsize', 12);
set(gca,'XTick',[]);
title('$Linear\;Function\;with\;Parabolic\;Blends\;(\ddot{\theta}=100\;deg/s^2)$','interpreter','LaTex','fontsize', 12);
legd1 = legend('Cubic Ploynomial', 'Linear&Parabolic');
set(legd1,'box','off');

H22 = subplot(3,1,2);
plot(t1, q_d1, 'linewidth',2)
hold on
plot(t3, q_d3_2, 'color', 'r', 'linewidth', 2)
ylabel('$\dot{\theta}$','interpreter','LaTex','fontsize',12);
set(gca,'XTick',[]); 
legd2 = legend('Cubic Ploynomial', 'Linear&Parabolic');
set(legd2,'box','off');

H23 = subplot(3,1,3);
plot(t1, q_dd1, 'linewidth',2)
hold on
plot(t3, q_dd3_2, 'color', 'r', 'linewidth', 2)
ylabel('$\ddot{\theta}$','interpreter','LaTex','fontsize',12);
xlabel('\it{Time (s)}','fontname','Times','fontsize',12);
legd3 = legend('Cubic Ploynomial', 'Linear&Parabolic');
set(legd3,'box','off');
