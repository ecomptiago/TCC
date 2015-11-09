listenerNode = robotics.ros.Node('Listener'); 
sub = robotics.ros.Subscriber(listenerNode,'/chatter', 'std_msgs/String', @testCallbackHelloWorld);
rosshutdown();

