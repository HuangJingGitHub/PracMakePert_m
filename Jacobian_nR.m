function J = Jacobian_nR(linkNum, q, linkLength)
%%% Jacobian for plannr n-link revolute robot %%%
    J = zeros(2, linkNum);
    sum_q = zeros(linkNum, 1);
    sum_q(1) = q(1);
    for i = 2:linkNum
        sum_q(i) = sum_q(i-1) + q(i);
    end
    
    for column = 1:linkNum
        for j = column:linkNum 
        J(:, column) = J(:, column) + [-linkLength(j) * sin(sum_q(j)); 
                                        linkLength(j) * cos(sum_q(j))];
        end
    end
end
