/*
 * BaseRosNode.h
 *
 *  Created on: Aug 22, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_BASEROSNODE_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_BASEROSNODE_H_

#include <ros/ros.h>

class BaseRosNode {
	protected:
		char *nodeName;
		ros::NodeHandle nodeHandler;
	public:
		BaseRosNode(int argc, char **argv,char* nodeName);
		virtual ~BaseRosNode();
		virtual int main(int argc, char **argv);
};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_BASEROSNODE_H_ */
