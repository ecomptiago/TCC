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
