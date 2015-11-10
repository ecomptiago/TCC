classdef AvoidObstaclesController <  common.src.BaseRosNode 
    
    properties
        laserValues
    end
    
    methods
        %Constructor
        function avoidObstaclesInstance = AvoidObstaclesController 
            avoidObstaclesInstance = avoidObstaclesInstance@...
                common.src.BaseRosNode...
            (avoid_obstacles.src.constants.AvoidObstaclesConstants.nodeName);
        end
        
        
        function subscribeToTopics(instance)
           addSubscribedTopic(instance, ...
               avoid_obstacles.src.constants.AvoidObstaclesConstants.laserTopic,...
               avoid_obstacles.src.constants.AvoidObstaclesConstants.laserTopicMsgType, ...
               @instance.laserTopicCallbackFunction);  
        end
        
        function createPublishers(instance)
            addPublisherClient(instance, ...
                avoid_obstacles.src.constants.AvoidObstaclesConstants.cmdVelTopic,...
                avoid_obstacles.src.constants.AvoidObstaclesConstants.cmdVelTopicMsgType);
            addPublisherClient(instance, ...
                avoid_obstacles.src.constants.AvoidObstaclesConstants.helloWorldTopic,...
                avoid_obstacles.src.constants.AvoidObstaclesConstants.helloWorldTopicMsgType);
            
        end
        
        %Callback functions
        function laserTopicCallbackFunction(instance,~,msg)
            instance.laserValues = msg.Ranges;
        end
        
        function runNode(instance)
            helloWorldPublisher = instance.publisherMap...
                (avoid_obstacles.src.constants.AvoidObstaclesConstants.helloWorldTopic);
            message = rosmessage(helloWorldPublisher);
            message.Data = 'Hello world';
            while(1)
                pause(1);
                helloWorldPublisher.send(message);
                disp(instance.laserValues);
            end
        end
    end
end