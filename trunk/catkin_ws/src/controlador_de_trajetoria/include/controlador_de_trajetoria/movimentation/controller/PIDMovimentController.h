/*
 * PIDController.h
 *
 *  Created on: Oct 2, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_PIDCONTROLLER_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_PIDCONTROLLER_H_
#define VREP_SIMULATION

#include "math.h"
#include "MovimentControllerInterface.h"

#ifdef VREP_SIMULATION
	#include "geometry_msgs/PoseStamped.h"
#else
	#include "nav_msgs/Odometry.h"
#endif

#include "ros/console.h"
#include "controlador_de_trajetoria/Position.h"
#include "common/utils/OdometryUtils.h"

class PIDMovimentController : public MovimentControllerInterface {

	private:
		//Attributes
		float kRho;
		float kAlpha;
		float kBeta;
		float rho;
		float alpha;
		float beta;
		controlador_de_trajetoria::Position targetPosition;

		//TODO - Use shared_ptr instead of raw pointer
		const controlador_de_trajetoria::Position *pointerTargetPosition;

	public:
		//Constructor
		PIDMovimentController();
		PIDMovimentController(float kRho, float kAlpha, float kBeta);

		//Destructor
		virtual ~PIDMovimentController() {};

		//Setters and getters
		void setTargetPosition(
			const controlador_de_trajetoria::Position& targetPosition);

		//Methods
		virtual geometry_msgs::Twist calculateVelocities();
		virtual float calculateError();

		#ifdef VREP_SIMULATION
			void calculateRhoAlphaBeta(geometry_msgs::PoseStamped actualOdometryPosition);
		#else
			void calculateRhoAlphaBeta(nav_msgs::Odometry actualOdometryPosition);
		#endif

};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_PIDCONTROLLER_H_ */
