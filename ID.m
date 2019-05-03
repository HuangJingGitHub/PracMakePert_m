function tau = ID(q_t, q_dt, q_ddt, linkNum, h, Ii)           
%%% Inverse Dynamics using RNEA in link coordinates and spatial vector algebra %%%
    tau = zeros(linkNum, 1);
    
    v = zeros(6, linkNum+1);
    a = zeros(6, linkNum+1);
    a(6, 1) = 9.8;
    f = zeros(6, linkNum+1);

    [X_M, ~]  = spatialTrans(q_t(1), [0, 0, 0]');
    v(:, 2)   = X_M * v(:, 1) + h * q_dt(1);
    a(:, 2)   = X_M * a(:, 1) + spatialCrossM(v(:, 2), h) * q_dt(1)+ h * q_ddt(1);
    f(:, 2)   = Ii * a(:, 2) + spatialCrossF(v(:, 2), Ii*v(:, 2));    
    for n = 2:linkNum
        [X_M, ~]  = spatialTrans(q_t(n), [0, 0.1, 0]');
        v(:, n+1)   = X_M * v(:, n) + h * q_dt(n);
        a(:, n+1)   = X_M * a(:, n) + spatialCrossM(v(:, n+1), h) * q_dt(n)+ h * q_ddt(n);
        f(:, n+1)   = Ii * a(:, n+1) + spatialCrossF(v(:, n+1), Ii*v(:, n+1));
    end
    for n = linkNum+1:-1:2
        [~, X_F]    = spatialTrans(q_t(n-1), [0, 0.1, 0]');
        tau(n-1) = h'*f(:, n);
        f(:, n-1)   = f(:, n-1) + X_F \ f(:, n);
    end 
end
