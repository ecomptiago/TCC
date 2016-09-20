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
                (avoid_obstacles.src.constants.AvoidObstaclesConstants...
                .nodeName);
            avoidObstaclesInstance.constants = ...
                avoid_obstacles.src.constants.AvoidObstaclesConstants;
            avoidObstaclesInstance.fuzzySystem = ...
                readfis('fuzzy_desvio_objeto.fis');
        end
        
        %Methods
        function subscribeToTopics(instance)
           addSubscribedTopic(instance, instance.constants.laserTopic,...
               instance.constants.laserTopicMsgType, ...
               @instance.laserTopicCallbackFunction);
        end
        
        function createPublishers(instance)
            addPublisherClient(instance, instance.constants.turnAngleTopic,...
                instance.constants.turnAngleTopicMsgType);
        end
        
        function runNode(instance)
            while(1)
                matrix30x6 = reshape(instance.laserValues,[],6);
                meanValues = mean(matrix30x6,'double');
                meanValues(meanValues > 3.5) = 3.5;
                turnRate = evalfis(meanValues,instance.fuzzySystem);
                turnAngleMsg = ... 
                    rosmessage(instance.constants.turnAngleTopicMsgType);
                turnAngleMsg.Data = turnRate;
                send(instance.publisherMap(instance.constants...
                    .turnAngleTopic),turnAngleMsg);
                pause(0.5);
            end
        end


        %Callback
        function laserTopicCallbackFunction(instance,~,msg)
            instance.laserValues = msg.Ranges;
        end
        
    end
end
