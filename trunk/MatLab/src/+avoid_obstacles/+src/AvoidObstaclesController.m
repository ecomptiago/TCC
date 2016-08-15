classdef AvoidObstaclesController <  common.src.BaseRosNode 
    
    properties
        laserValues
        constants
        fuzzySystem
        robotPose
        robotAngle
        rulesArray
        rulesNumber
        rulesActivationCounter
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
            avoidObstaclesInstance.rulesNumber = ... 
                size(avoidObstaclesInstance.fuzzySystem.rule);
            avoidObstaclesInstance.rulesNumber = avoidObstaclesInstance...
                .rulesNumber(2);
            avoidObstaclesInstance.rulesActivationCounter = ...
                zeros(1,avoidObstaclesInstance.rulesNumber);
            avoidObstaclesInstance.rulesArray = avoid_obstacles...
                    .src.FuzzyRule.empty(0,avoidObstaclesInstance...
                    .rulesNumber);
            for i = 1.0:avoidObstaclesInstance.rulesNumber
                avoidObstaclesInstance.rulesArray(i) = avoid_obstacles...
                    .src.FuzzyRule(avoidObstaclesInstance.fuzzySystem...
                    .rule(i).antecedent,avoidObstaclesInstance.fuzzySystem...
                    .input);
            end
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
        
        function rotateLeft(instance)
            send(instance.publisherMap(instance.constants.cmdVelTopic),...
                instance.createCmdVelMsg(0,0.1));
        end
        
        function rotateRight(instance)
            send(instance.publisherMap(instance.constants.cmdVelTopic),...
                instance.createCmdVelMsg(0,-0.1));
        end
        
        function runNode(instance)
            instance.stop();
            while(1)
                matrix30x6 = reshape(instance.laserValues,[],6);
                meanValues = mean(matrix30x6,'double');
                meanValues(meanValues > 3.5) = 3.5;
                turnRate = evalfis(meanValues,instance.fuzzySystem);
                if(instance.robotPose < 9.75) 
                    fprintf('robotAngle: %f\n', instance.robotAngle);
                    fprintf('turnRate %f \n', turnRate / 100);
                    if(instance.robotAngle < 0 && instance.robotAngle > -90) 
                        instance.rotateLeft();
                        while(~(instance.robotAngle<110 && instance.robotAngle ...
                            >90)) 
                            fprintf('robotAngle while: %f\n', instance.robotAngle);
                            pause(0.1);
                        end        
                    elseif(instance.robotAngle > -180 && instance.robotAngle ...
                        < -130) 
                            instance.rotateRight();
                            while(~(instance.robotAngle<110 && instance.robotAngle ...
                            >90)) 
                                fprintf('robotAngle while: %f\n', instance.robotAngle);
                                pause(0.1);
                            end
                    elseif(turnRate >= 0.1 || turnRate <= -0.1) 
                         instance.turn(turnRate / 100);
%                          for i=1:instance.rulesNumber
%                             activated = instance.rulesArray(1,i)...
%                                 .wasActivated(meanValues);
%                             if(activated)
%                                 instance.rulesActivationCounter(1,i) = ...
%                                     instance.rulesActivationCounter(1,i) ...
%                                     + 1;
%                             end
%                          end
                    else 
                         instance.moveForward();
                    end
                else
                    instance.stop();
                end
                  disp(instance.rulesActivationCounter);  
                  pause(0.5);
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
            instance.robotAngle = rad2deg(angle(3));
        end 
    end
end
