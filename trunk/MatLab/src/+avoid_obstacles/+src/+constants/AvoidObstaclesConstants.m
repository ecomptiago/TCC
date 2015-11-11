classdef AvoidObstaclesConstants
    properties(Constant)
        nodeName = 'Avoid_obstacles_controller';
       
        %Topics
        laserTopic =  'vrep/laser';
        laserTopicMsgType = 'sensor_msgs/LaserScan';
        cmdVelTopic = 'RosAria/cmd_vel';
        cmdVelTopicMsgType = 'geometry_msgs/Twist';
        
        %Services

    end
end

