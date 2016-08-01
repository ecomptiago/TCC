classdef AvoidObstaclesConstants
    properties(Constant)
        
        nodeName = 'Avoid_obstacles_controller';
        constLinearVel = 0.1;
        
        %Topics
        laserTopic =  'RosAria/laser';
        laserTopicMsgType = 'sensor_msgs/LaserScan';
        cmdVelTopic = 'RosAria/cmd_vel';
        cmdVelTopicMsgType = 'geometry_msgs/Twist';
        poseTopic = 'RosAria/pose';
        poseTopicMsgType = 'geometry_msgs/PoseStamped';
        
        %Services
        
    end
end

