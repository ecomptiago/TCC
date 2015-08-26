/*
 * BaseRosNode.h
 *
 *  Created on: Aug 22, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_BASEROSNODE_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_BASEROSNODE_H_

#include "string"
#include "ros/ros.h"
#include "controlador_de_trajetoria/RosNodeInterface.h"

class BaseRosNode : public RosNodeInterface{

	public:
		//Constructors
		BaseRosNode(int argc, char **argv, std::string nodeName);

		//Destructor
		virtual ~BaseRosNode() {};

		//Setters and getters

		//Methods
		virtual int runNode();

};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_BASEROSNODE_H_ */
