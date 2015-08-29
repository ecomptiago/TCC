/*
 * BaseRosNode.cpp
 *
 *  Created on: Aug 22, 2015
 *      Author: tiago
 */

#include "controlador_de_trajetoria/BaseRosNode.h"

//Constructors
BaseRosNode::BaseRosNode(int argc, char **argv, std::string nodeName) {
	this->pointerToNode = NULL;

	ROS_INFO("Initializing ROS node %s",nodeName.c_str());
	ros::init(argc, argv, nodeName);
}

//Getters and setters
const std::string BaseRosNode::getNodeName() {
	MethodNotImplementedError error(__func__,"BaseRosNode");
}

//Methods
int BaseRosNode::runNode() {
	MethodNotImplementedError error(__func__,"BaseRosNode");
}

BaseRosNode*& BaseRosNode::getPointerToNode() {
	return pointerToNode;
}

void BaseRosNode::setPointerToNode(BaseRosNode* pointerToNode) {
	this->pointerToNode = pointerToNode;
}
