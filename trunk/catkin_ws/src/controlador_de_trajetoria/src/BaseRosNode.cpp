/*
 * BaseRosNode.cpp
 *
 *  Created on: Aug 22, 2015
 *      Author: tiago
 */

#include "controlador_de_trajetoria/BaseRosNode.h"

//Constructors
BaseRosNode::BaseRosNode(int argc, char **argv, std::string nodeName) {
	ROS_INFO("Initializing ROS node %s",nodeName.c_str());
	ros::init(argc, argv, nodeName);
	this->angleErrorMargin = 0.75;
	this->positionErrorMargin = 0.25;
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

