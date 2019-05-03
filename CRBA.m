function M = CRBA(linkNum, Ii, hi, q_t, linkLength)   %%% Composite-Rigid-Body Algorithm
    fci = zeros(6, linkNum, linkNum);
    Ici = zeros(6, 6, linkNum);    
    M = zeros(linkNum, linkNum);

    for i = 1:linkNum
        Ici(:,:,i) = Ii;
    end
    
    for i = linkNum:-1:1
        fci(:, i, i) = Ici(:,:,i) * hi;
        for j = i:linkNum
            M(i, j) = hi' * fci(:,i, j);
            M(j, i) = M(i, j);
        end
        if i ~= 1
            %%% Special attention to the vector argument given to spatialTrans(). 
            [X_M, X_F] = spatialTrans(-q_t(i), [0 -linkLength(i)*cos(q_t(i)) linkLength(i)*sin(q_t(i))]');
            Ici(:,:,i-1) = Ici(:,:,i-1) + X_F * Ici(:,:,i) / X_M;
            for j = i:linkNum
                fci(:, i-1, j) = X_F * fci(:, i, j);
            end
        end
    end
end