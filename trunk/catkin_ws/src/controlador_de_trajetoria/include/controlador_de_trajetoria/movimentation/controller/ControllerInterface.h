/*
 * ControllerInterface.h
 *
 *  Created on: Oct 2, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_CONTROLLER_CONTROLLERINTERFACE_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_CONTROLLER_CONTROLLERINTERFACE_H_

#include "geometry_msgs/Twist.h"

class ControllerInterface {
	public:
		//Destructor
		virtual ~ControllerInterface() {};

		//Methods
		virtual geometry_msgs::Twist calculateVelocities() = 0;
		virtual float calculateError() = 0;
};


#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_CONTROLLER_CONTROLLERINTERFACE_H_ */
