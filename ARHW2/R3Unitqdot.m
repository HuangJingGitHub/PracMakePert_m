function qdot = R3Unitqdot()
pra1 = linspace(0,2*pi,50);
pra2 = 0:0.05:1;
idx = 1;
qdot = zeros(3,2*length(pra1)*length(pra2));
for i = 1:length(pra1)
    for j = 1:length(pra2)
        qdot(1,idx) = pra2(j)*cos(pra1(i));
        qdot(2,idx) = pra2(j)*sin(pra1(i));
        qdot(3,idx) = sqrt(1-pra2(j)^2);
        idx = idx + 1;
        qdot(1,idx) = pra2(j)*cos(pra1(i));
        qdot(2,idx) = pra2(j)*sin(pra1(i));
        qdot(3,idx) = -sqrt(1-pra2(j)^2);
        idx = idx+1;
    end
end
end