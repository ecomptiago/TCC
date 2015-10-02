/*
 * PIDController.cpp
 *
 *  Created on: Oct 2, 2015
 *      Author: tiago
 */

#include "controlador_de_trajetoria/movimentation/controller/PIDController.h"

PIDController::PIDController() {
	this->kRho = 0;
	this->kAlpha = 0;
	this->kBeta = 0;
}

PIDController::PIDController(float kRho, float kAlpha, float kBeta) {
	this->kRho = kRho;
	this->kAlpha = kAlpha;
	this->kBeta = kBeta;
}

geometry_msgs::Twist PIDController::calculateVelocities() {
	ROS_INFO("ABC");

}

float PIDController::calculateError() {
	ROS_INFO("ABC");
	return 0.1;
}
