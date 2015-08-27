/*
 * BaseRosNode.cpp
 *
 *  Created on: Aug 22, 2015
 *      Author: tiago
 */

#include "stdexcept"
#include "string"
#include "typeinfo"
#include "ros/ros.h"
#include "ros/exceptions.h"
#include "controlador_de_trajetoria/BaseRosNode.h"
#include "controlador_de_trajetoria/error/MethodNotImplementedError.h"

//Constructors
BaseRosNode::BaseRosNode(int argc, char **argv, std::string nodeName) {
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
