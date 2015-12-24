/*
 * pathPlanner.cpp
 *
 *  Created on: Dec 15, 2015
 *      Author: tiago
 */

#include "path_planner/PathPlanner.h"

//Constructors
PathPlanner::PathPlanner(int argc, char **argv) :
	BaseRosNode(argc, argv, nodeName){
}

//Methods
int PathPlanner::runNode() {
	ROS_INFO("Running node");

	ros::spin();
	BaseRosNode::shutdownAndExit(nodeName);
}


bool PathPlanner::subscribeToTopics() {
	ROS_INFO("Subscribing to topics");
	return true;
}

bool PathPlanner::createServices() {
	ROS_INFO("Creating services");
	if(createServiceClients()) {
		return createServiceServers();
	} else {
		return false;
	}
}

bool PathPlanner::createServiceClients() {
	return addServiceClient<path_planner::simRosGetObjectGroupData>(nodeHandler, getObjectGroupDataService);
}

bool PathPlanner::createServiceServers() {
	return true;
}

int PathPlanner::infoFailAndExit(const char* topicName) {
	ROS_INFO("Failed to create %s topic",topicName);
	return BaseRosNode::shutdownAndExit(nodeName);
}

bool PathPlanner::createPublishers() {
	ROS_INFO("Creating publishers");
	return true;
}

//Callback

//Main
int main(int argc, char **argv) {
	PathPlanner rosAriaVRep(argc,argv);
	try{
		if(rosAriaVRep.createServices() &&
			rosAriaVRep.subscribeToTopics() &&
			rosAriaVRep.createPublishers()) {
				return rosAriaVRep.runNode();
		} else {
			BaseRosNode::shutdownAndExit(nodeName);
		}
	} catch (std::exception &e) {
		BaseRosNode::shutdownAndExit(nodeName);
	}
}
