/*
 * PositionHandler.h
 *
 *  Created on: Aug 25, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_POSITION_POSITIONHANDLER_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_POSITION_POSITIONHANDLER_H_

#include "string"
#include "controlador_de_trajetoria/BaseRosNode.h"

//Constants
const std::string nodeName = "Position_handler";

class PositionHandler :public BaseRosNode{
	private:
		//Attributes
		ros::NodeHandle nodeHandler;

	public:
		//Constructors
		PositionHandler(int argc,char **argv);

		//Destructor
		virtual ~PositionHandler() {} ;

		//Getters and setters
		const std::string getNodeName();

		//Methods
		virtual int runNode();
};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_POSITION_POSITIONHANDLER_H_ */
