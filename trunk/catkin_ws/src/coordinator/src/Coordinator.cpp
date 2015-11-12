/*
 * Coordinator.cpp
 *
 *  Created on: Nov 12, 2015
 *      Author: tcoelho
 */

#include "coordinator/Coordinator.h"

//Constructor
Coordinator::Coordinator(int argc, char **argv) :
	BaseRosNode(argc, argv, nodeName){
}

//Methods
int Coordinator::runNode() {
	ROS_INFO("Running node");
	ros::spin();
	BaseRosNode::shutdownAndExit(nodeName);
}

bool Coordinator::subscribeToTopics() {
	ROS_INFO("Subscribing to topics");
	return addSubscribedTopic<const sensor_msgs::LaserScan::ConstPtr&, Coordinator>(nodeHandler,laserTopic,
		&Coordinator::receivedLaserValues,this);
}

//Callback
void Coordinator::receivedLaserValues(
	const sensor_msgs::LaserScan::ConstPtr& laserReading) {
	ROS_INFO("%f",
		std::max_element(laserReading->ranges.begin(), laserReading->ranges.end()));
}

//Main
int main(int argc, char **argv) {
	Coordinator coordinator(argc,argv);
	try{
		if(coordinator.subscribeToTopics()) {
			return coordinator.runNode();
		} else {
			BaseRosNode::shutdownAndExit(nodeName);
		}
	} catch (std::exception &e) {
		BaseRosNode::shutdownAndExit(nodeName);
	}
}
