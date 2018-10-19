function J = R3Jac(q)
    s1 = q(1);
    s12 = q(1) + q(2);
    s123 = q(1) + q(2) + q(3);
    J = [-sin(s1)-sin(s12)-sin(s123) -sin(s12)-sin(s123) -sin(123);
          cos(s1)+cos(s12)+cos(s123) cos(s12)+cos(s123) cos(s123)];
end