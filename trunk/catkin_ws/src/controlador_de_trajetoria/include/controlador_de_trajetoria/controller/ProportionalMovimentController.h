/*
 * PIDController.h
 *
 *  Created on: Oct 2, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_PROPORTIONALMOVIMENTCONTROLLER_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_PROPORTIONALMOVIMENTCONTROLLER_H_
#define VREP_SIMULATION

#include "math.h"
#include "MovimentControllerInterface.h"

#ifdef VREP_SIMULATION
	#include "geometry_msgs/PoseStamped.h"
#else
	#include "nav_msgs/Odometry.h"
#endif

#include "ros/console.h"
#include "common/Position.h"
#include "common/utils/OdometryUtils.h"
#include "common/utils/NumericUtils.h"

class ProportionalMovimentController : public MovimentControllerInterface {

	private:
		//AttributesDougkla
		float kRho;
		float kAlpha;
		float kBeta;
		float rho;
		float alpha;
		float beta;
		common::Position targetPosition;

		//TODO - Use shared_ptr instead of raw pointer
		const common::Position *pointerTargetPosition;

	public:
		//Constructor
		ProportionalMovimentController();
		ProportionalMovimentController(float kRho, float kAlpha, float kBeta);

		//Destructor
		virtual ~ProportionalMovimentController() {};

		//Setters and getters
		void setTargetPosition(
			const common::Position& targetPosition);

		//Methods
		virtual geometry_msgs::Twist calculateVelocities();
		virtual float calculateError();

		#ifdef VREP_SIMULATION
			void calculateRhoAlphaBeta(geometry_msgs::PoseStamped actualOdometryPosition);
		#else
			void calculateRhoAlphaBeta(nav_msgs::Odometry actualOdometryPosition);
		#endif

};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_PROPORTIONALMOVIMENTCONTROLLER_H_ */
