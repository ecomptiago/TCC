%*********************************************************
%   Usar average e resharp para calcular as medias!!!!!!!!!!
%*********************************************************    
classdef AvoidObstaclesController <  common.src.BaseRosNode 
    
    properties
        laserValues
        constants
    end
    
    methods
        %Constructor
        function avoidObstaclesInstance = AvoidObstaclesController 
            avoidObstaclesInstance = ...
                avoidObstaclesInstance@common.src.BaseRosNode...
                (avoid_obstacles.src.constants.AvoidObstaclesConstants.nodeName);
            avoidObstaclesInstance.constants = ...
                avoid_obstacles.src.constants.AvoidObstaclesConstants;
        end
        
        function subscribeToTopics(instance)
           addSubscribedTopic(instance, instance.constants.laserTopic,...
               instance.constants.laserTopicMsgType, ...
               @instance.laserTopicCallbackFunction);  
        end
        
        function createPublishers(instance)
            addPublisherClient(instance, instance.constants.cmdVelTopic,...
                instance.constants.cmdVelTopicMsgType);
        end
        
        function runNode(~)
            while(1)
                pause(1);
            end
        end
        
        %Callback functions
        function laserTopicCallbackFunction(instance,~,msg)
            instance.laserValues = msg.Ranges;
        end
        
    end
end