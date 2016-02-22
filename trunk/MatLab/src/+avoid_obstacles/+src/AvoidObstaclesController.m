classdef AvoidObstaclesController <  common.src.BaseRosNode 
    
    properties
        laserValues
        constants
        fuzzySystem
        robotPose
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
           addSubscribedTopic(instance,instance.constants.poseTopic,...
               instance.constants.poseTopicMsgType, ...
               @instance.poseTopicCallbackFunction);
        end
        
        function createPublishers(instance)
            addPublisherClient(instance, instance.constants.cmdVelTopic,...
                instance.constants.cmdVelTopicMsgType);
        end
        
        function runNode(instance)
%             cmdVelMsg = rosmessage(instance.constants.cmdVelTopicMsgType);
%             cmdVelMsg.Linear.X = 0.1;
%             cmdVelMsg.Angular.Z = -0.1;
%             send(instance.publisherMap(instance.constants.cmdVelTopic),cmdVelMsg);
            while(1)
%                 matrix30x6 = reshape(instance.laserValues,[],6);
%                 meanValues =mean(matrix30x6,'double');
%                 disp(meanValues);
%                 meanValues(meanValues > 3.5) = 3.5;
%                 disp(meanValues);
%                 turnRate = evalfis(meanValues,instance.fuzzySystem);
%when turnRate is positive we need to set a negative value to Angular.Z and positive otherwise
%                 disp(turnRate);
%                 disp(instance.robotPose);
                pause(1);
            end
        end
        
        %Callback functions
        function laserTopicCallbackFunction(instance,~,msg)
            instance.laserValues = msg.Ranges;
        end
        
        function poseTopicCallbackFunction(instance,~,msg)
            instance.robotPose = msg.Pose;
        end 
    end
end
