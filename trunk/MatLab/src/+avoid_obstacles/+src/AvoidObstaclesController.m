classdef AvoidObstaclesController <  common.src.BaseRosNode 
    
    properties
        laserValues
    end
    
    methods
        function avoidObstaclesInstance = AvoidObstaclesController 
            avoidObstaclesInstance = avoidObstaclesInstance@...
                common.src.BaseRosNode('Avoid_obstacles_controller');
        end
        
        function subscribeToTopics(instance)
           addSubscribedTopic(instance, ...
               avoid_obstacles.src.constants.AvoidObstaclesConstants.laserTopic,...
               avoid_obstacles.src.constants.AvoidObstaclesConstants.laserTopicMsgType, ...
               @instance.laserTopicCallbackFunction);  
        end
       
        function laserTopicCallbackFunction(instance,~,msg)
            instance.laserValues = msg;
        end
        
        function runNode(instance)
            while(1)
                pause(1);
                disp(instance.laserValues.Header.Stamp);
            end
        end
    end
end