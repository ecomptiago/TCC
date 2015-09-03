#include <controlador_de_trajetoria/messages/MessagesHandler.h>

//Constructors
MessagesHandler::MessagesHandler(int argc, char **argv) :
	BaseRosNode(argc,argv,nodeName) {
}

//Methods
int MessagesHandler::runNode() {
	ros::spin();
	return 0;
}


void MessagesHandler::proccessPositionToMoveRobot(
		const controlador_de_trajetoria::Move_robot::ConstPtr& moveRobotPosition) {
	ROS_INFO("Received Message");
}


bool MessagesHandler::subscribeToTopics() {
	ros::Subscriber sub = nodeHandler.subscribe(moveRobotAssyncTopic, 1000,
			&MessagesHandler::proccessPositionToMoveRobot,
			this);
	if(sub) {
		subscriberMap[moveRobotAssyncTopic] = sub;
		return true;
	} else {
		return false;
	}
}


bool MessagesHandler::createPublishers() {
	/*ros::Publisher pub = nodeHandler.advertise<std_msgs::String>(
			actualRobotPositionTopic, 1000);
	if(pub) {
		publisherMap[actualRobotPositionTopic] =  pub;
		return true;
	} else {
		return false;
	}*/
	return true;
}

//Main
int main(int argc,char **argv) {
	try {
		MessagesHandler messagesHandler(argc,argv);

		if(messagesHandler.subscribeToTopics() && messagesHandler.createPublishers()) {
			return messagesHandler.runNode();
		} else {
			ros::shutdown();
			return 0;
		}

	} catch (std::exception &e) {
		ros::shutdown();
		return 0;
	}
}
