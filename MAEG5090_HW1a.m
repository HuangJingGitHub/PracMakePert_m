%%% MAEG5090 Topics in Roboatics Dynamics Assignment1 1a %%%
%%% Inverse dynamics of the robot using Recrsive Newton-Euler Algorithm  
%%% presented in ''Robot Dynamics: Equation and Algorithms'' by Roy
%%% Featherstone and David Orin in 2000.
clear
close all
clc

%%% Set robot parameters.
linkNum  = 7;           % Set link number here.
links    = 1:linkNum;   % link series
linkMass = 0.2;         
Ic = diag([0.0005 0 0.0005]);
gravity = -9.8;

%%% Set simulation parameters.
timeLen     = 6;        
timeStep    = 0.001;
timeSeries  = 0:timeStep:timeLen;
stepNum     = length(timeSeries);

%%% Compute rigid body inertia. In the frame configuration, link frame is
%%% set at the strating end of each link.
r = [0 0.075 0]';
r_mat = vector2Mat(r);      % from vector to corresponding skew-symmetric matrix
Ii = zeros(6, 6);
Ii(1:3, 1:3) = Ic - linkMass * r_mat * r_mat;
Ii(1:3, 4:6) = linkMass * r_mat;
Ii(4:6, 1:3) = linkMass * r_mat';
Ii(4:6, 4:6) = linkMass * eye(3,3);

v = zeros(6, linkNum+1);    % spatial velocity of each link frame, including base frame {0}
a = zeros(6, linkNum+1);    % acceleration
a(6, 1) = -gravity;         % Initialize base acceleration of gravity.
f = zeros(6, linkNum+1);    % joint force 
tau = zeros(linkNum, stepNum);      % joint torque
h = [1 0 0 0 0 0]';         % h matrix

tic
for i = 1:stepNum
    %%% Get joint data.
    q_t    = 0.6 * sin( pi * timeSeries(i) + (links - 1) * pi/3 );
    q_dt   = 0.6 * pi * cos(pi * timeSeries(i) + (links - 1) * pi/3 );
    q_ddt  = -0.6 * pi * pi * sin(pi * timeSeries(i) + (links - 1) * pi/3);
    
    %%% from base to joint 1
    [X_M, X_F]  = spatialTrans(q_t(1), [0, 0, 0]');     % sptial transformation for motion and force vectors
    v(:, 2)   = X_M * v(:, 1) + h * q_dt(1);
    a(:, 2)   = X_M * a(:, 1) + spatialCrossM(v(:, 2), h) * q_dt(1)+ h * q_ddt(1);
    f(:, 2)   = Ii * a(:, 2) + spatialCrossF(v(:, 2), Ii*v(:, 2));
    
    %%% forward recursion 
    for n = 2:linkNum
        [X_M, X_F]  = spatialTrans(q_t(n), [0, 0.1, 0]');
        v(:, n+1)   = X_M * v(:, n) + h * q_dt(n);
        a(:, n+1)   = X_M * a(:, n) + spatialCrossM(v(:, n+1), h) * q_dt(n)+ h * q_ddt(n);
        f(:, n+1)   = Ii * a(:, n+1) + spatialCrossF(v(:, n+1), Ii*v(:, n+1));
    end
    %%% backward recursion
    for n = linkNum+1:-1:2
        [~, X_F]    = spatialTrans(q_t(n-1), [0, 0.1, 0]');
        tau(n-1, i) = h'*f(:, n);
        f(:, n-1)   = f(:, n-1) + X_F \ f(:, n);
    end 
end
runTime = toc
save('test_torque', 'tau');

figure('position',[120 95 950 550])
for i = 1:linkNum
    plot(timeSeries, tau(i,:),'LineWidth', 1);
    hold on
end
xlabel('time (s)','Fontname','Times', 'FontSize', 12);
ylabel('$\tau \; N{\cdot}m$', 'Interpreter', 'LaTex', 'FontSize',12);
leng1 = legend('${\tau}_1$', '${\tau}_2$', '${\tau}_3$', '${\tau}_4$', '${\tau}_5$',...
               '${\tau}_6$', '${\tau}_7$');
set(leng1, 'Interpreter', 'LaTex', 'Location', 'northeast', 'Orientation', 'horizontal');