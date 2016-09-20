classdef AvoidObstaclesConstants
    properties(Constant)
        
        nodeName = 'Avoid_obstacles_controller';
        
        %Topics
        laserTopic =  'RosAria/laser';
        laserTopicMsgType = 'sensor_msgs/LaserScan';
        turnAngleTopic = 'AvoidObstacles/turnAngle';
        turnAngleTopicMsgType = 'std_msgs/Float32';
        %Services
        
    end
end

