function [q, qd, qdd, X] = AngleCal(n, L1, L2, q0, dt, Len, x_d)
q   = zeros(n,Len);
qd  = zeros(n,Len);
qdd = zeros(n,Len);
X   = zeros(3,Len);

q(:,2) = q0;
for loop = 2:Len
    q1 = q(1,loop);                   % theta1
    q12 = q(1,loop) + q(2,loop);      % theta1+theta2
    
    %%% Calculate Jacobian matrix under each pose/time step
    J(1,1) = -L1*sin(q1) - L2*sin(q12);
    J(1,2) = -L2*sin(q12);              %J(1,1) + L1*sin(q1);
    J(2,1) = L1*cos(q1) + L2*cos(q12);
    J(2,2) = L2*cos(q12);               %J(1,1) - L1*cos(q1);
    
    qd(:,loop) = J\x_d;                % dot_ang = inv(J)*dot_x
    qdd(:,loop) = (qd(:,loop) - qd(:,loop-1))./dt;
    q(:,loop+1) = q(:,loop) + qd(:,loop)*dt;  
    % Suppose the commanded joint angles are perfectly achieved.
    X(:,loop) = [L1*cos(q1)+L2*cos(q12), L1*sin(q1)+L2*sin(q12),q12]';
end
q = q(:,1:Len);
end