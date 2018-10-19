function pinv = Wpinv(J,W)
%%% Weighted Pesudoinverse
    [U,S,V] = svd(J);
    pinv = W\V*S'/(S*V'/W*V*S')*U';  % pinv = inv(W)*J'*inv((J*inv(W)*J'))
end