function x = tr2vec(T)
%%% To convert transformation matrix to vector representation.

R = T(1:3, 1:3);
theta = acos((trace(R) - 1) / 2);
K = [ R(3, 2) - R(2, 3);
      R(1, 3) - R(3, 1);
      R(2, 1) - R(1, 2) ];

epsilon = 1e-5;
if theta > epsilon && theta < pi - epsilon     
    K = theta * K / (2*sin(theta));
else
    K = theta * K / 1e-10;
end

x = [T(1:3,4); K];

end