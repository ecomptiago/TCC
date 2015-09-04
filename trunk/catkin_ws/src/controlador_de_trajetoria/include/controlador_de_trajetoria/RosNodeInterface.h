/*
 * RosNodeInterface.h
 *
 *  Created on: Aug 26, 2015
 *      Author: cinq
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_ROSNODEINTERFACE_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_ROSNODEINTERFACE_H_

#include "string"

class RosNodeInterface {
	public:
		//Destructor
		virtual ~RosNodeInterface() {};

		//Setters and getters

		//Methods
		virtual int runNode() = 0;
		virtual bool subscribeToTopics() = 0;
		virtual bool createPublishers() = 0;
};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_ROSNODEINTERFACE_H_ */
