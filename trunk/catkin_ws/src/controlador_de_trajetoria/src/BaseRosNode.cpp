/*
 * BaseRosNode.cpp
 *
 *  Created on: Aug 22, 2015
 *      Author: tiago
 */

#include "stdexcept"
#include "string"
#include "ros/ros.h"
#include "ros/exceptions.h"
#include "controlador_de_trajetoria/BaseRosNode.h"

//Constructors
BaseRosNode::BaseRosNode(int argc, char **argv, std::string nodeName) {
	ROS_INFO("Initializing ROS node %s",nodeName.c_str());
	ros::init(argc, argv, nodeName);
}

//Getters and setters

//Methods
int BaseRosNode::runNode() {
	throw new std::runtime_error("This method was not implemented");
}
