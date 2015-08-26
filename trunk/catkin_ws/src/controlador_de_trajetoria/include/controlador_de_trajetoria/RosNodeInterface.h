/*
 * RosNodeInterface.h
 *
 *  Created on: Aug 26, 2015
 *      Author: cinq
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_ROSNODEINTERFACE_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_ROSNODEINTERFACE_H_

class RosNodeInterface {
	public:
		//Destructor
		virtual ~RosNodeInterface() {};

		//Methods
		virtual int runNode() = 0;
};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_ROSNODEINTERFACE_H_ */
