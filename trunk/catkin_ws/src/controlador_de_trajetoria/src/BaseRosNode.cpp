/*
 * BaseRosNode.cpp
 *
 *  Created on: Aug 22, 2015
 *      Author: tiago
 */

#include <controlador_de_trajetoria/BaseRosNode.h>
#include <ros/ros.h>
#include <stdexcept>

BaseRosNode::BaseRosNode(int argc, char **argv, char *nodeName) {
	this->nodeName = nodeName;

	ros::init(argc, argv, nodeName);
	ROS_INFO("Initialized ROS node %s",nodeName);
}

BaseRosNode::~BaseRosNode() {
	ROS_INFO("Shutting down ROS node %s",nodeName);
	nodeHandler.shutdown();
}

int::main(int argc, char **argv) {
	throw new std::exception("This method was not implemented");
}
