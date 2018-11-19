function [a, b, c] = testf(val)
a = val * eye(3);
b = 2 * val * eye(3);
c = 3 * val * eye(3);
end