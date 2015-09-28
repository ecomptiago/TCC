/*
 * BaseRosNode.cpp
 *
 *  Created on: Aug 22, 2015
 *      Author: tiago
 */

#include "../include/common/BaseRosNode.h"

//Constructors
BaseRosNode::BaseRosNode(int argc, char **argv, std::string nodeName) {
	ROS_INFO("Initializing ROS node %s",nodeName.c_str());
	ros::init(argc, argv, nodeName);
	//TODO - Verify what parameters are good for vrep simulation
	this->angleErrorMargin = 0.75;
	this->positionErrorMargin = 0.25;
	this->defaultQueueSize = 100;
}

//Methods
int BaseRosNode::runNode() {
	MethodNotImplementedError error(__func__,"BaseRosNode");
}

bool BaseRosNode::subscribeToTopics() {
	MethodNotImplementedError error(__func__,"BaseRosNode");
}

bool BaseRosNode::createPublishers() {
	MethodNotImplementedError error(__func__,"BaseRosNode");
}

bool BaseRosNode::createTimers() {
	MethodNotImplementedError error(__func__,"BaseRosNode");
}

bool BaseRosNode::createServices() {
	MethodNotImplementedError error(__func__,"BaseRosNode");
}

bool BaseRosNode::hasPublisher(const char* topicName) {
	std::map<std::string,ros::Publisher>::iterator i = publisherMap.find(topicName);
	if(i != publisherMap.end()) {
		return true;
	} else {
		ROS_DEBUG("%s was not found",topicName);
		return false;
	}
}

void BaseRosNode::sleepAndSpin(double miliSeconds) {
	usleep(miliSeconds * 1000);
	ros::spinOnce();
}

void BaseRosNode::sleepAndSpin(ros::Rate& rate) {
	rate.sleep();
	ros::spinOnce();
}

int BaseRosNode::shutdownAndExit(const char* nodeName) {
	ros::shutdown();
	ROS_INFO("ROS node %s is being shuttled down",nodeName);
	return 0;
}