/*
 * Coordinator.cpp
 *
 *  Created on: Nov 12, 2015
 *      Author: tcoelho
 */

#include "coordinator/Coordinator.h"

//Constructor
Coordinator::Coordinator(int argc, char **argv) :
	BaseRosNode(argc, argv, "Coordinator"){
}

//Methods
int Coordinator::runNode() {
	ROS_INFO("Running node");
	ros::Rate rate(1/0.75);
	while(ros::ok()) {
		if(laserValues.capacity() != 0 && NumericUtils::isFirstGreaterEqual<float>(proportionalError.data,0.2)) {
			float smallestLaserReading = *std::min_element(laserValues.begin(),
				laserValues.end());
			ROS_DEBUG("The smallest element is %f",smallestLaserReading);
			if(NumericUtils::isFirstLessEqual<float>(smallestLaserReading, 0.5)) {
				geometry_msgs::Twist move;
				move.linear.x = 0.75 * smallestLaserReading;
				ROS_DEBUG("fuzzyTurnAngle %f",fuzzyTurnAngle.data);
				move.angular.z = 0.02 * fuzzyTurnAngle.data ;
				publisherMap[cmdVelTopic].publish(move);
				ROS_DEBUG("Setting velocity liner %f and angular %f",move.linear.x,move.angular.z);
			} else {
				publisherMap[cmdVelTopic].publish(proportionalVelocity);
				ROS_DEBUG("Setting velocity liner %f and angular %f",
					proportionalVelocity.linear.x,proportionalVelocity.angular.z);
			}
		} else {
			geometry_msgs::Twist stop;
			stop.angular.x = 0;
			stop.angular.y = 0;
			stop.angular.z = 0;
			stop.linear.x = 0;
			stop.linear.y = 0;
			stop.linear.z = 0;
			publisherMap[cmdVelTopic].publish(stop);
			ROS_DEBUG("Setting velocity liner 0 and angular 0");
		}

		sleepAndSpin(rate);
	}
	return shutdownAndExit();
}

bool Coordinator::subscribeToTopics() {
	ROS_INFO("Subscribing to topics");
	return addSubscribedTopic<const sensor_msgs::LaserScan::ConstPtr&, Coordinator>(nodeHandler,laserTopic,
		&Coordinator::receivedLaserValues,this) &&

		addSubscribedTopic<const geometry_msgs::Twist::ConstPtr&,Coordinator>(nodeHandler,velTopic,
			&Coordinator::receivedProportionalControlerVelocity,this) &&

		addSubscribedTopic<const std_msgs::Float32::ConstPtr&,Coordinator>(nodeHandler,errorTopic,
			&Coordinator::receivedProportionalControlerError,this) &&

		addSubscribedTopic<const geometry_msgs::PoseStamped::ConstPtr&,Coordinator>(nodeHandler,poseTopic,
			&Coordinator::receivedRobotPose,this) &&

		addSubscribedTopic<const std_msgs::Float32::ConstPtr&,Coordinator>(nodeHandler,turnAngleTopic,
			&Coordinator::receivedFuzzyTurnAngle,this);

}

bool Coordinator::createPublishers() {
	ROS_INFO("Creating publishers");
	return addPublisherClient<common::Move_robot>(
		nodeHandler, targetPositionTopic, false) &&

		addPublisherClient<geometry_msgs::Twist>(
			nodeHandler,cmdVelTopic,false);
}


//Callback
void Coordinator::receivedLaserValues(
	const sensor_msgs::LaserScan::ConstPtr& laserReading) {
		if(laserValues.capacity() == 0 ||
			laserReading->ranges.capacity()	!= laserValues.capacity()) {
			laserValues.resize(laserReading->ranges.capacity());
		}
		std::copy(laserReading->ranges.begin(),laserReading->ranges.end(),
			laserValues.begin());
}

void Coordinator::receivedProportionalControlerVelocity(
	const geometry_msgs::Twist::ConstPtr& proportionalVelocity) {
		this->proportionalVelocity.angular = proportionalVelocity->angular;
		this->proportionalVelocity.linear = proportionalVelocity->linear;
}

void Coordinator::receivedProportionalControlerError(
	const std_msgs::Float32::ConstPtr& proportionalError) {
		this->proportionalError.data = proportionalError->data;
}

void Coordinator::receivedRobotPose(
	const geometry_msgs::PoseStamped::ConstPtr& robotPose){
		this->robotPose.header = robotPose->header;
		this->robotPose.pose = robotPose->pose;
}

void Coordinator::receivedFuzzyTurnAngle(
	const std_msgs::Float32::ConstPtr& fuzzyTurnAngle){
		this->fuzzyTurnAngle.data = fuzzyTurnAngle->data;
}

//Main
int main(int argc, char **argv) {
	Coordinator coordinator(argc,argv);
	try{
		if(coordinator.subscribeToTopics() &&
			coordinator.createPublishers()) {
				return coordinator.runNode();
		} else {
			 return coordinator.shutdownAndExit();
		}
	} catch (std::exception &e) {
		return coordinator.shutdownAndExit(e);
	}
}
