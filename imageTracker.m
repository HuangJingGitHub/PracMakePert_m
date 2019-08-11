classdef imageTracker < handle
    properties(SetAccess = immutable)
        imageTrackerROSNameSpace;
        imageTrackerROSSubscriber;
    end
    
    properties(SetAccess = protected)
        imageFeatureData;
    end
    
    methods
        function self = imageTracker(ROSTopicName)
            if nargin == 0
                disp('ROS topic name is needed to initialize imageTracker.');
                return
            end
            
            self.imageTrackerROSSubscriber = rossubscriber(ROSTopicName, rostype.sensor_msgs_PointCloud2);
        end
        
        function featureData = getFeatureData(self)
            msg = self.imageTrackerROSSubscriber.LatestMessage;
            
            if size(msg.Data, 1) == 0
                disp('No valid feature in this image.');
                return 
            end
            
            featureData = readXYZ(msg);
            self.imageFeatureData = featureData;
        end
        
    end
end