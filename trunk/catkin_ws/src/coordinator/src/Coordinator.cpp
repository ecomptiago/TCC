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
		this->minLaserValue = 0;
		this->maxLaserValue = 0;
}

//Methods
int Coordinator::runNode() {
	ROS_INFO("Running node");
	ros::spin();
	BaseRosNode::shutdownAndExit(nodeName);
}

bool Coordinator::isInLaserRange(const float& laserValue) {
	return !(NumericUtils::isFirstLessEqual<float>(laserValue,maxLaserValue) &&
		NumericUtils::isFirstGreaterEqual<float>(laserValue,minLaserValue));
}

bool Coordinator::subscribeToTopics() {
	ROS_INFO("Subscribing to topics");
	return addSubscribedTopic<const sensor_msgs::LaserScan::ConstPtr&, Coordinator>(nodeHandler,laserTopic,
		&Coordinator::receivedLaserValues,this);
}

//Callback
void Coordinator::receivedLaserValues(
	const sensor_msgs::LaserScan::ConstPtr& laserReading) {
	maxLaserValue = laserReading->range_max;
	minLaserValue = laserReading->range_min;
	std::copy(laserReading->ranges.begin(),laserReading->ranges.end(),
		laserValues.begin());
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
