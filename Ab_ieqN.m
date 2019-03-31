function [A_ieqN, b_ieqN] = Ab_ieqN(A, B, C, R, x_0, Z_REF, boundary, N)
    E = [1 0; -1 0; 0 1; 0 -1];
    A_ieq = E * R * C * B;
    A_ieqN = cell(N, N);
    b_ieqN = cell(N, 1);
    for i = 1:N
        for j = 1:N
            A_ieqN{i, j} = zeros(size(A_ieq));
            b_ieqN{i, 1} = boundary - E * R *(C*A*x_0 - Z_REF(2*j-1:2*j, 1));
        end
        A_ieqN{i, i} = A_ieq;
    end
    A_ieqN = cell2mat(A_ieqN);
    b_ieqN = cell2mat(b_ieqN);
end