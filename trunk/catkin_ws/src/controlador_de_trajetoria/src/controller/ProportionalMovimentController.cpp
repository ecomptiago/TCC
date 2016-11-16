/*
 * ProportionalMovimentController.cpp
 *
 *  Created on: Oct 2, 2015
 *      Author: tiago
 */

#include "controlador_de_trajetoria/controller/ProportionalMovimentController.h"

//Constructor
ProportionalMovimentController::ProportionalMovimentController() {
	this->kRho = 0;
	this->kAlpha = 0;
	this->kBeta = 0;
	this->rho = 0;
	this->alpha = 0;
	this->beta = 0;
	this->pointerTargetPosition = NULL;
	this->theta = 0;
	this->angleToGoal = 0;
}

ProportionalMovimentController::ProportionalMovimentController(float kRho, float kAlpha,
	float kBeta) {
		float kAlphaMinValue = ((5/3) *kBeta) - ((2/M_PI) * kRho);
		if(kRho < 0 || kBeta > 0 || kAlpha < kAlphaMinValue) {
			this->kRho = 0;
			this->kAlpha = 0;
			this->kBeta = 0;
			ROS_ERROR("kRho needs to be > 0, kBeta needs to be < 0, "
				"kAlpha need to be > %f", kAlphaMinValue);
		} else {
			this->kRho = kRho;
			this->kAlpha = kAlpha;
			this->kBeta = kBeta;
		}
		this->rho = 0;
		this->alpha = 0;
		this->beta = 0;
		this->pointerTargetPosition = NULL;
		this->theta = 0;
		this->angleToGoal = 0;

}

//Setters and getters
void ProportionalMovimentController::setTargetPosition(
	const common::Position& targetPosition) {
		this->targetPosition = targetPosition;
		this->pointerTargetPosition = &targetPosition;
}

const geometry_msgs::Twist ProportionalMovimentController::calculateVelocities() {
	geometry_msgs::Twist twist;
	if(NumericUtils::isFirstGreater<float>(alpha, -M_PI / 2) &&
		NumericUtils::isFirstLessEqual<float>(alpha, M_PI / 2)) {
			twist.linear.x = kRho * rho;
			twist.angular.z =
				(kAlpha * alpha) + (kBeta * beta);
	}
//	else if((NumericUtils::isFirstGreater<float>(alpha,-M_PI) &&
//		NumericUtils::isFirstLessEqual<float>(alpha, -M_PI/2)) ||
//		(NumericUtils::isFirstGreater<float>(alpha,M_PI/2) &&
//		NumericUtils::isFirstLessEqual<float>(alpha, M_PI))){
//			twist.linear.x = -kRho * rho;
//			twist.angular.z =
//				-(kAlpha * alpha) - (kBeta * beta);
//	}
	else {
		twist.linear.x = 0;
		double robotAngle;
		double angleToGoalTemp;
		if(NumericUtils::isFirstLess<double>(theta,0)) {
			robotAngle = theta + (2 * M_PI);
		} else {
			robotAngle = theta;
		}
		if(NumericUtils::isFirstLess<double>(angleToGoal,0)) {
			angleToGoalTemp = angleToGoal + (2 * M_PI);
		} else {
			angleToGoalTemp = angleToGoal;
		}
		if(NumericUtils::isFirstLess<float>(angleToGoalTemp,robotAngle)) {
			double angleBetweenRobotGoal =
				robotAngle - angleToGoalTemp;
			double angleBetweenRobotGoal2 =
				(2 * M_PI) - angleBetweenRobotGoal;
			ROS_DEBUG("angleToGoalTemp: %f , robotAngle: %f , angleBetweenRobotGoal: %f , angleBetweenRobotGoal2: %f",
				angleToGoalTemp * 180/M_PI, robotAngle * 180/M_PI, angleBetweenRobotGoal* 180/M_PI,
				angleBetweenRobotGoal2 * 180/M_PI);
			if(NumericUtils::isFirstLess<double>(angleBetweenRobotGoal,
				angleBetweenRobotGoal2)) {
					twist.angular.z = -0.3;
			} else {
				twist.angular.z = 0.3;
			}
		} else {
			double angleBetweenRobotGoal =
				angleToGoalTemp - robotAngle;
			double angleBetweenRobotGoal2 =
				(2 * M_PI) - angleBetweenRobotGoal;
			ROS_DEBUG("angleToGoalTemp: %f , robotAngle: %f , angleBetweenRobotGoal: %f , angleBetweenRobotGoal2: %f",
				angleToGoalTemp * 180/M_PI, robotAngle * 180/M_PI, angleBetweenRobotGoal* 180/M_PI,
				angleBetweenRobotGoal2 * 180/M_PI);
			if(NumericUtils::isFirstLess<double>(angleBetweenRobotGoal,
				angleBetweenRobotGoal2)) {
					twist.angular.z = 0.3;
			} else {
				twist.angular.z = -0.3;
			}
		}
	}
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
			theta = OdometryUtils::getAngleFromQuaternation
				(quaternion.normalize(),true);
			ROS_DEBUG("Calculated deltaX:%f deltaY:%f theta:%f",
				deltaX, deltaY, theta);
			angleToGoal = atan2(deltaY, deltaX);
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
				OdometryUtils::getAngleFromQuaternation(quaternion.normalize());
			ROS_DEBUG("Calculated deltaX:%f deltaY:%f theta:%f",
				deltaX, deltaY, theta);
			this->rho = sqrt(pow(deltaX , 2) + pow(deltaY, 2));
			this->alpha = -theta + (atan2(deltaY, deltaX) * 180/M_PI);
			ROS_DEBUG("Calculated rho:%f alpha:%f ", rho, alpha);
	}
#endif
