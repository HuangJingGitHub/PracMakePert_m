classdef retractor < psm %%
   properties (SetAccess = immutable)
       deltaT = 0.001;
       camera2PSM = eye(4);
   end
   
   methods
       function self = retractor(name)
           self@psm(name);
       end
   end
end
