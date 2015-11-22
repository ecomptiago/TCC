classdef AvoidObstaclesController <  common.src.BaseRosNode 
    
    properties
        laserValues
        constants
        fuzzySystem
    end
    
    methods
        %Constructor
        function avoidObstaclesInstance = AvoidObstaclesController 
            avoidObstaclesInstance = ...
                avoidObstaclesInstance@common.src.BaseRosNode...
                (avoid_obstacles.src.constants.AvoidObstaclesConstants.nodeName);
            avoidObstaclesInstance.constants = ...
                avoid_obstacles.src.constants.AvoidObstaclesConstants;
            avoidObstaclesInstance.fuzzySystem = ...
                readfis('../fuzzy_desvio_objeto.fis');
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
        
        function runNode(instance)
            while(1)
                matrix30x6 = reshape(instance.laserValues,[],6);
                meanValues =mean(matrix30x6,'double');
                disp(meanValues);
                meanValues(meanValues > 3.5) = 3.5;
                disp(meanValues);
                turnRate = evalfis(meanValues,instance.fuzzySystem);
                disp(turnRate);
                pause(1);
            end
        end
        
        %Callback functions
        function laserTopicCallbackFunction(instance,~,msg)
            instance.laserValues = msg.Ranges;
        end
        
    end
end