
#include "controlador_de_trajetoria/position/PositionHandler.h"

//Constructors
PositionHandler::PositionHandler(int argc, char **argv) :
	BaseRosNode(argc,argv,getNodeName()) {
}

//Getters and setters
const std::string PositionHandler::getNodeName() {
	return "Position_handler";
}

//Methods
int PositionHandler::runNode() {
	ros::spin();
	ros::shutdown();
	return 0;
}

bool PositionHandler::subscribeToTopics() {
	PositionHandler* pointerToPositionHandler =
			(PositionHandler*) (getPointerToNode());

	char * topicName = (char *)"chatter";
	uint32_t messages = 1000;

	void (PositionHandler::*callback)(const std_msgs::String::ConstPtr& message) =
					&PositionHandler::callback;

	ros::Subscriber sub = nodeHandler.subscribe(topicName, messages,
			callback, pointerToPositionHandler);
	return sub;
}

void PositionHandler::callback(const std_msgs::String::ConstPtr& message) {
	ROS_INFO("ABC");
}

//Main
int main(int argc,char **argv) {
	try {
		PositionHandler positionHandler(argc,argv);

		positionHandler.setPointerToNode(&positionHandler);

		if(positionHandler.subscribeToTopics()) {
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
