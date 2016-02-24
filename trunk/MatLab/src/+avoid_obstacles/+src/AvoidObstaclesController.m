classdef AvoidObstaclesController <  common.src.BaseRosNode 
    
    properties
        laserValues
        constants
        fuzzySystem
        robotPose
        robotAngleX
        robotAngleY
        robotAngleZ
        robotTestAngle
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
        
        %Methods
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
        
        function cmdVelMsg = createCmdVelMsg(instance,linearVel,angularVel) 
            cmdVelMsg = rosmessage(instance.constants.cmdVelTopicMsgType);
            cmdVelMsg.Linear.X = linearVel;
            cmdVelMsg.Angular.Z = angularVel;
        end 
        
        function moveForward(instance)
            send(instance.publisherMap(instance.constants.cmdVelTopic),...
                instance.createCmdVelMsg(instance.constants.constLinearVel,...
                0));
        end
        
        function stop(instance)
            send(instance.publisherMap(instance.constants.cmdVelTopic),...
                instance.createCmdVelMsg(0,0));
        end
        
        function turn(instance, turnRate)
            send(instance.publisherMap(instance.constants.cmdVelTopic),...
                instance.createCmdVelMsg(instance.constants.constLinearVel,...
                turnRate));
        end
        
        function runNode(instance)
             instance.stop();
%             instance.moveForward();
            while(1)
%                 matrix30x6 = reshape(instance.laserValues,[],6);
%                 meanValues = mean(matrix30x6,'double');
%                 meanValues(meanValues > 3.5) = 3.5;
%                 turnRate = evalfis(meanValues,instance.fuzzySystem);
%                 if(turnRate >= 0.1 || turnRate <= -0.1)
%                     disp(turnRate / 75);
%                     instance.turn(turnRate / 75);
%                 elseif(instance.robotPose < 10) 
%                     instance.moveForward();
%                 else
%                     instance.stop();
%                 end
                  fprintf('X: %f Y: %f Z: %f\n', ...
                      rad2deg(instance.robotAngleX), ...
                      rad2deg(instance.robotAngleY),...
                      rad2deg(instance.robotAngleZ));
                  pause(0.75);
            end
        end
        
        %Callback
        function laserTopicCallbackFunction(instance,~,msg)
            instance.laserValues = msg.Ranges;
        end
        
        function poseTopicCallbackFunction(instance,~,msg)
            instance.robotPose = msg.Pose.Position.Y;
            angle = quat2eul(quatnormalize([0, ...
                0, msg.Pose.Orientation.Z, ...
                msg.Pose.Orientation.W]));
            instance.robotAngleX = angle(1);
            instance.robotAngleY = angle(2);
            instance.robotAngleZ = angle(3);
        end 
    end
end
