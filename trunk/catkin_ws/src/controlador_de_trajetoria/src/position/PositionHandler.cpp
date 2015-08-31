
#include "controlador_de_trajetoria/position/PositionHandler.h"

//Constructors
PositionHandler::PositionHandler(int argc, char **argv) :
	BaseRosNode(argc,argv,nodeName) {
}

//Methods
int PositionHandler::runNode() {
	ros::spin();
	return 0;
}

void PositionHandler::transformOdometryToPosition(
		const nav_msgs::Odometry::ConstPtr& odometryPosition) {
	ROS_INFO("Received a message from pose topic");
	ROS_DEBUG("Processing odometry message");

	ROS_DEBUG("Publishing position message");

	std::map<std::string,ros::Publisher>::iterator i = publisherMap.find(actualRobotPositionTopic);
	if(i != publisherMap.end()) {
		std_msgs::String msg;
		msg.data = "position in x,y";
		i->second.publish(msg);
	} else {
		ROS_DEBUG("actual_robot_position was not found");
	}
}

bool PositionHandler::subscribeToTopics() {
	ros::Subscriber sub = nodeHandler.subscribe(poseTopic, 1000,
			&PositionHandler::transformOdometryToPosition,
			this);
	if(sub) {
		subscriberMap[poseTopic] = sub;
		return true;
	} else {
		return false;
	}
}


bool PositionHandler::createPublishers() {
	ros::Publisher pub = nodeHandler.advertise<std_msgs::String>(
			actualRobotPositionTopic, 1000);
	if(pub) {
		publisherMap[actualRobotPositionTopic] =  pub;
		return true;
	} else {
		return false;
	}
}

//Main
int main(int argc,char **argv) {
	try {
		PositionHandler positionHandler(argc,argv);

		if(positionHandler.subscribeToTopics() && positionHandler.createPublishers()) {
			return positionHandler.runNode();
		} else {
			ros::shutdown();
			return 0;
		}

	} catch (std::exception &e) {
		ros::shutdown();
		return 0;
	}
}
