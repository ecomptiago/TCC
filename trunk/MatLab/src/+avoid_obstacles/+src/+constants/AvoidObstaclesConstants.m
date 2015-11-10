classdef AvoidObstaclesConstants
    properties(Constant)
        nodeName = 'Avoid_obstacles_controller';
        laserTopic =  'vrep/laser';
        laserTopicMsgType = 'sensor_msgs/LaserScan';
        cmdVelTopic = 'RosAria/cmd_vel';
        cmdVelTopicMsgType = 'geometry_msgs/Twist';
        helloWorldTopic = 'HelloWorldTopic'
        helloWorldTopicMsgType = 'std_msgs/String';
    end
end

