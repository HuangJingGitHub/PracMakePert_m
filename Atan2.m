%%% Adjusted atan(). The value range is extended to(-pi,pi] according to 
%%% the signs of the 2 attributes y and x. The default return unit is degree.
%%% Refer to https://en.wikipedia.org/wiki/Atan2 for more info.
function ang = Atan2(y,x,angUnit)
if x==0 && y>0
   ang = 90;
elseif x==0 && y<0
   ang = -90;
elseif x==0 && y==0
   ang = NaN;
elseif x>0 && y>=0
    ang = atan(y/x)/pi*180;
elseif x<0 && y>=0
    ang = atan(y/x)/pi*180 + 180;
elseif x<0 && y<0
    ang = atan(y/x)/pi*180 - 180;
elseif x>0 && y<0
    ang = atan(y/x)/pi*180;
end

if nargin == 3 && strcmp(angUnit, 'rad')
    ang = ang / 180 * pi;
end
