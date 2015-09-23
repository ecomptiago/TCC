
#include "controlador_de_trajetoria/position/PositionHandler.h"

//Constructors
PositionHandler::PositionHandler(int argc, char **argv, float actualRobotPositionDelay) :
	BaseRosNode(argc,argv,nodeName) {
	this->actualRobotPositionDelay = actualRobotPositionDelay;
}

//Methods
int PositionHandler::runNode() {
	ROS_INFO("Running node");
	ros::spin();
	return 0;
}

bool PositionHandler::subscribeToTopics() {
	ROS_INFO("Subscribing to topics");
	return addSubscribedTopic<const nav_msgs::Odometry::ConstPtr&, PositionHandler>(nodeHandler,poseTopic,
			&PositionHandler::transformOdometryToPosition,this);
}

bool PositionHandler::createPublishers() {
	ROS_INFO("Creating publishers");
	return addPublisherClient<controlador_de_trajetoria::Position>(nodeHandler, actualRobotPositionTopic, false);
}

bool PositionHandler::createTimers() {
	ROS_INFO("Creating timers");
	return addTimer<const ros::TimerEvent&, PositionHandler>(nodeHandler, actualRobotPositionDelay,
		   timerActualRobotPosition, &PositionHandler::publishPosition, this, false);
}

//Callbacks
void PositionHandler::transformOdometryToPosition(
	const nav_msgs::Odometry::ConstPtr& odometryPosition) {
		ROS_DEBUG("Received position x:%f y:%f",odometryPosition->pose.pose.position.x,
			odometryPosition->pose.pose.position.y);
		position.x = odometryPosition->pose.pose.position.x;
		position.y = odometryPosition->pose.pose.position.y;
}

void PositionHandler::publishPosition(const ros::TimerEvent& timerEvent) {
	if(hasPublisher(actualRobotPositionTopic)) {
		publisherMap[actualRobotPositionTopic].publish(position);
	}
}

//Main
int main(int argc,char **argv) {
	try {
		PositionHandler positionHandler(argc,argv,1);
		if(positionHandler.subscribeToTopics() &&
				positionHandler.createPublishers() &&
				positionHandler.createTimers()) {
			return positionHandler.runNode();
		} else {
			BaseRosNode::shutdownAndExit(nodeName);
		}
	} catch (std::exception &e) {
		BaseRosNode::shutdownAndExit(nodeName);
	}
}
