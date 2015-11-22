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
	ros::Rate rate(1);
	while(ros::ok()) {
		if(laserValues.capacity() != 0) {
			float smallestLaserReading = *std::min_element(laserValues.begin(),
				laserValues.end());
			std::cout << "The smallest element is " << smallestLaserReading << '\n';
			if(NumericUtils::isFirstLessEqual<float>(smallestLaserReading, 3.5)) {
				std::cout<<"Trigger fuzzy \n";
			}
		}
		sleepAndSpin(rate);
	}
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
	if(laserValues.capacity() == 0 ||
		laserReading->ranges.capacity()	!= laserValues.capacity()) {
		laserValues.resize(laserReading->ranges.capacity());
	}
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
