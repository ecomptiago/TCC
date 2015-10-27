/*
 * ProportionalMovimentController.cpp
 *
 *  Created on: Oct 2, 2015
 *      Author: tiago
 */

#include "../../../include/controlador_de_trajetoria/movimentation/controller/ProportionalMovimentController.h"

//Constructor
ProportionalMovimentController::ProportionalMovimentController() {
	this->kRho = 0;
	this->kAlpha = 0;
	this->kBeta = 0;
	this->rho = 0;
	this->alpha = 0;
	this->beta = 0;
	this->pointerTargetPosition = NULL;
}

ProportionalMovimentController::ProportionalMovimentController(float kRho, float kAlpha, float kBeta) {
	this->kRho = kRho;
	this->kAlpha = kAlpha;
	this->kBeta = kBeta;
	this->rho = 0;
	this->alpha = 0;
	this->beta = 0;
	this->pointerTargetPosition = NULL;
}

//Setters and getters
void ProportionalMovimentController::setTargetPosition(
	const controlador_de_trajetoria::Position& targetPosition) {
		this->targetPosition = targetPosition;
		this->pointerTargetPosition = &targetPosition;
}

geometry_msgs::Twist ProportionalMovimentController::calculateVelocities() {
	geometry_msgs::Twist twist;
	twist.linear.x = kRho * rho;
	twist.angular.z =
		(kAlpha * alpha) + (kBeta * beta);
	ROS_DEBUG("Setting velocities linear:%f angular:%f", twist.linear.x,
		twist.angular.z);
	return twist;
}

float ProportionalMovimentController::calculateError() {
	float error = rho;
	ROS_DEBUG("Error: %f",error);
	return error;
}

#ifdef VREP_SIMULATION
	void ProportionalMovimentController::calculateRhoAlphaBeta(
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
				(quaternion.normalize(),true);
			ROS_DEBUG("Calculated deltaX:%f deltaY:%f theta:%f",
				deltaX, deltaY, theta);
			double angleToGoal = atan2(deltaY, deltaX);
			rho = sqrt(pow(deltaX , 2) + pow(deltaY, 2));
			alpha = -theta + angleToGoal;
			beta = -theta - alpha;
			ROS_DEBUG("Calculated rho:%f alpha:%f beta:%f", rho, alpha, beta);
	}
#else
	void ProportionalMovimentController::calculateRhoAlphaBeta(
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
			ROS_DEBUG("Calculated rho:%f alpha:%f ", rho, alpha);
	}
#endif
