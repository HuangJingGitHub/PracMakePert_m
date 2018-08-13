function ang = Atan2(y,x,angunit)
% Adjusted atan(). The value range is extended to(-pi,pi] according to 
% the signs of the 2 attributes y and x.

if x==0 && y>0
   ang = 90;
elseif x==0 && y<0
   ang = -90;
elseif x==0 && y==0
   ang = NaN;
end
  
if x>0 && y>=0
    ang = atan(y/x)/pi*180;
elseif x<0 && y>=0
    ang = atan(y/x)/pi*180 + 180;
elseif x<0 && y<0
    ang = atan(y/x)/pi*180 - 180;
elseif x>0 && y<0
    ang = atan(y/x)/pi*180;
end

if nargin == 3 && strcmp(angunit, 'rad')
    ang = ang / 180 * pi;
end
