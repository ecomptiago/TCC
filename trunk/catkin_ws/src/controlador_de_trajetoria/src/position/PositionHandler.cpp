
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
	ros::Subscriber sub = nodeHandler.subscribe(poseTopic, 1000,
			&PositionHandler::transformOdometryToPosition,
			this);
	if(sub) {
		subscriberMap[poseTopic] = sub;
		return true;
	} else {
		ROS_DEBUG("Could not subscribe to all topics");
		return false;
	}
}

bool PositionHandler::createPublishers() {
	ROS_INFO("Creating publishers");
	ros::Publisher pub =
		nodeHandler.advertise<controlador_de_trajetoria::Position>(
		actualRobotPositionTopic, 1000);
	if(pub) {
		publisherMap[actualRobotPositionTopic] =  pub;
		return true;
	} else {
		ROS_DEBUG("Could not create all publishers");
		return false;
	}
}

bool PositionHandler::createTimers() {
	ROS_INFO("Creating timers");
	ros::Timer timer =
			nodeHandler.createTimer(ros::Duration(actualRobotPositionDelay),
				&PositionHandler::publishPosition,
				this,false);
	if(timer) {
		timerMap[actualRobotPositionTopic] = timer;
		return true;
	} else {
		ROS_DEBUG("Could not create all timers");
		return false;
	}
}

//Callbacks
void PositionHandler::transformOdometryToPosition(
		const nav_msgs::Odometry::ConstPtr& odometryPosition) {
	ROS_INFO("Received a message from pose topic");

	ROS_DEBUG("Processing odometry message");
	position.x = odometryPosition->pose.pose.position.x;
	position.y = odometryPosition->pose.pose.position.y;
}


void PositionHandler::publishPosition(const ros::TimerEvent& timerEvent) {
	ROS_DEBUG("Publishing position message");
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
			ros::shutdown();
			return 0;
		}

	} catch (std::exception &e) {
		ros::shutdown();
		return 0;
	}
}
