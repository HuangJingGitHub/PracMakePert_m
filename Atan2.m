function ang = Atan2(x,y)
%Adjusted atan() function
if x==0 && y>0
    ang = inf;
elseif x==0 && y<0
    ang = -inf;
elseif x==0 && y==0
    ang = NaN;
end

if x>0 && y>0
    ang = atan(y/x)/pi*180;
elseif x<0 && y>0
    ang = atan(y/x)/pi*180 + 180;
elseif x<0 && y<0
    ang = atan(y/x)/pi*180 - 180;
elseif x>0 && y<0
    ang = atan(y/x)/pi*180;
end
