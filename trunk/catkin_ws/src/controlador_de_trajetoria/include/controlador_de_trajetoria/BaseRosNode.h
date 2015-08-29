/*
 * BaseRosNode.h
 *
 *  Created on: Aug 22, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_BASEROSNODE_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_BASEROSNODE_H_

#include "stdexcept"
#include "ros/ros.h"
#include "ros/exceptions.h"
#include "controlador_de_trajetoria/RosNodeInterface.h"
#include "controlador_de_trajetoria/error/MethodNotImplementedError.h"


class BaseRosNode : public RosNodeInterface{

	protected:
		//Attributes
		BaseRosNode *pointerToNode;

	public:
		//Constructors
		BaseRosNode(int argc, char **argv, std::string nodeName);

		//Destructor
		virtual ~BaseRosNode() {};

		//Setters and getters
		virtual const std::string getNodeName();
		BaseRosNode*& getPointerToNode();
		void setPointerToNode(BaseRosNode* pointerToNode);

		//Methods
		virtual int runNode();
};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_BASEROSNODE_H_ */
