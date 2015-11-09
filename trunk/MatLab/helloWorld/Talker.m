talkerNode = robotics.ros.Node('/Talker');
pub = robotics.ros.Publisher(talkerNode,'/chatter','std_msgs/String');
msg = rosmessage(pub);
count = 0;
while(count < 1000)
    msg.Data = strcat('Hello world ', num2str(count));
    send(pub,msg);
    count = count + 1;
    pause(1);
end
rosshutdown();
