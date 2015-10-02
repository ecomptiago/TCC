/*
 * PIDController.h
 *
 *  Created on: Oct 2, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_PIDCONTROLLER_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_PIDCONTROLLER_H_

#include "ControllerInterface.h"
#include "ros/console.h"

class PIDController : public ControllerInterface {

	private:
		//Attributes
		float kRho;
		float kAlpha;
		float kBeta;

	public:
		//Constructor
		PIDController();
		PIDController(float kRho, float kAlpha, float kBeta);

		//Destructor
		virtual ~PIDController() {};

		//Methods
		virtual geometry_msgs::Twist calculateVelocities();
		virtual float calculateError();

};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_PIDCONTROLLER_H_ */
