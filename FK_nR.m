function x = FK_nR(linkNum, q, linkLength)
    x = zeros(2, 1);
    sum_q = 0;
    
    for i = 1:linkNum
        sum_q = sum_q + q(i);
        x = x + linkLength(i) * [cos(sum_q); sin(sum_q)];
    end
end