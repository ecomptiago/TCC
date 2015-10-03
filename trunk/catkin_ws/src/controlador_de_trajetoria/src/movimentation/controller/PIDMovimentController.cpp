/*
 * PIDController.cpp
 *
 *  Created on: Oct 2, 2015
 *      Author: tiago
 */

#include "../../../include/controlador_de_trajetoria/movimentation/controller/PIDMovimentController.h"

//Constructor
PIDMovimentController::PIDMovimentController() {
	this->kRho = 0;
	this->kAlpha = 0;
	this->kBeta = 0;
	this->rho = 0;
	this->alpha = 0;
	this->beta = 0;
	this->pointerTargetPosition = NULL;
}

PIDMovimentController::PIDMovimentController(float kRho, float kAlpha, float kBeta) {
	this->kRho = kRho;
	this->kAlpha = kAlpha;
	this->kBeta = kBeta;
	this->rho = 0;
	this->alpha = 0;
	this->beta = 0;
	this->pointerTargetPosition = NULL;
}

//Setters and getters
void PIDMovimentController::setTargetPosition(
	const controlador_de_trajetoria::Position& targetPosition) {
		this->targetPosition = targetPosition;
		this->pointerTargetPosition = &targetPosition;
}

geometry_msgs::Twist PIDMovimentController::calculateVelocities() {
	double alpha = alpha * M_PI / 180;
	float newRho =
		-kRho * rho * cos(alpha);
	float newAlpha =
		(kRho * sin(alpha)) - (kAlpha * alpha) - (kBeta * beta);
	float newBeta = -kRho * sin(alpha);
	geometry_msgs::Twist twist;
	twist.linear.x = kRho * newRho;
	twist.angular.z =
		(kAlpha * newAlpha) + (kBeta * newBeta);
	return twist;
}

float PIDMovimentController::calculateError() {
	return rho + alpha + beta;
}

#ifdef VREP_SIMULATION
	void PIDMovimentController::calculateRhoAlphaBeta(
		geometry_msgs::PoseStamped actualOdometryPosition) {
			double deltaX =
				targetPosition.x - actualOdometryPosition.pose.position.x;
			double deltaY =
				targetPosition.y - actualOdometryPosition.pose.position.y;
			tf::Quaternion quaternion(
				0, 0, actualOdometryPosition.pose.orientation.z,
				actualOdometryPosition.pose.orientation.w);
			double theta =
				OdometryUtils::getAngleFromQuaternation<tf::Quaternion>
				(quaternion.normalize());
			ROS_DEBUG("Calculated deltaX:%f deltaY:%f theta:%f",
				deltaX, deltaY, theta);
			this->rho = sqrt(pow(deltaX , 2) + pow(deltaY, 2));
			this->alpha = -theta + (atan2(deltaY, deltaX) * 180/M_PI);
			this->beta = -theta - this->alpha;
			ROS_DEBUG("Calculated rho:%f alpha:%f beta:%f", rho, alpha,
				beta);
	}
#else
	void PIDMovimentController::calculateRhoAlphaBeta(
		nav_msgs::Odometry actualOdometryPosition) {
			double deltaX =
				targetPosition.x - actualOdometryPosition.pose.pose.position.x;
			double deltaY =
				targetPosition.y - actualOdometryPosition.pose.pose.position.y;
			tf::Quaternion quaternion(
				0, 0, actualOdometryPosition.pose.orientation.z,
				actualOdometryPosition.pose.pose.orientation.w);
			double theta =
				OdometryUtils::getAngleFromQuaternation<tf::Quaternion>
				(quaternion.normalize());
			ROS_DEBUG("Calculated deltaX:%f deltaY:%f theta:%f",
				deltaX, deltaY, theta);
			this->rho = sqrt(pow(deltaX , 2) + pow(deltaY, 2));
			this->alpha = -theta + (atan2(deltaY, deltaX) * 180/M_PI);
			this->beta = -theta - this->alpha;
			ROS_DEBUG("Calculated rho:%f alpha:%f beta:%f", rho, alpha,
				beta);
	}
#endif
