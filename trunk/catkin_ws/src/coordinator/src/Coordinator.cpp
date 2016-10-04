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
		triedToFindPath = false;
		reachedFinalGoal = false;
		recalculatePath =- false;
		pathPosition = -1;
}

//Methods
int Coordinator::runNode() {
	ROS_INFO("Running node");
	ros::Rate rate(1/0.75);
	while(ros::ok()) {
		sleepAndSpin(rate);
		float smallestLaserReading = *std::min_element(laserValues.begin(),
			laserValues.end());
		common::pathToTarget pathToTarget;
		pathToTarget.request.x = -1.57;
		pathToTarget.request.y = 9.6;
		if(!reachedFinalGoal) {
			if(!triedToFindPath ||
				(recalculatePath && NumericUtils::isFirstGreater<float>(smallestLaserReading, 0.5))) {
					serviceClientsMap[bestPathService].call(pathToTarget);
					pathPosition = 0;
					recalculatePath = false;
			} else {
				common::Position position;
				if(pathToTarget.response.path.size() == 0) {
					position.x = pathToTarget.request.x;
					position.y = pathToTarget.request.y;
					publisherMap[targetPositionTopic].publish(position);
				} else {
					common::cellGridPosition cellGridPosition;
					cellGridPosition.request.gridPosition = pathToTarget.response.path[pathPosition];
					serviceClientsMap[cellGridPositionService].call(cellGridPosition);
					common::Position position;
					position.x = cellGridPosition.response.x;
					position.y = cellGridPosition.response.y;
					publisherMap[targetPositionTopic].publish(position);
				}
			}
		}
		if(NumericUtils::isFirstGreaterEqual<float>(proportionalError.data,0.2)) {
			ROS_DEBUG("The smallest element is %f",smallestLaserReading);
			if(NumericUtils::isFirstLessEqual<float>(smallestLaserReading, 0.5)) {
				geometry_msgs::Twist move;
				move.linear.x = 0.75 * smallestLaserReading;
				ROS_DEBUG("fuzzyTurnAngle %f",fuzzyTurnAngle.data);
				move.angular.z = 0.02 * fuzzyTurnAngle.data ;
				publisherMap[cmdVelTopic].publish(move);
				ROS_DEBUG("Setting velocity liner %f and angular %f",move.linear.x,move.angular.z);
				recalculatePath = true;
			} else {
				publisherMap[cmdVelTopic].publish(proportionalVelocity);
				ROS_DEBUG("Setting velocity liner %f and angular %f",
					proportionalVelocity.linear.x,proportionalVelocity.angular.z);
			}
		} else {
			if(!reachedFinalGoal) {
				geometry_msgs::Twist stop;
				stop.angular.x = 0;
				stop.angular.y = 0;
				stop.angular.z = 0;
				stop.linear.x = 0;
				stop.linear.y = 0;
				stop.linear.z = 0;
				if(pathToTarget.response.path.size() == 0) {
					publisherMap[cmdVelTopic].publish(stop);
					ROS_DEBUG("Setting velocity liner 0 and angular 0");
				} else {
					if(pathPosition == pathToTarget.response.path.size()) {
						publisherMap[cmdVelTopic].publish(stop);
						ROS_DEBUG("Setting velocity liner 0 and angular 0");
					} else {
						pathPosition++;
					}
				}
			}
		}

		geometry_msgs::PoseStamped rvizPose;

		rvizPose.header = robotPose.header;
		rvizPose.pose = robotPose.pose;
		rvizPose.header.frame_id = "LaserScannerBody_2D";

		publisherMap[rvizPoseTopic].publish(rvizPose);

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
	return addPublisherClient<common::Position>(
		nodeHandler, targetPositionTopic, false) &&

		addPublisherClient<geometry_msgs::Twist>(
			nodeHandler,cmdVelTopic,false) &&

		addPublisherClient<geometry_msgs::PoseStamped>(
			nodeHandler,rvizPoseTopic,false);
}

bool Coordinator::createServices() {
	return addServiceClient<common::pathToTarget>(nodeHandler,bestPathService) &&

		addServiceClient<common::cellGridPosition>(nodeHandler,cellGridPositionService);
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
			coordinator.createPublishers() &&
			coordinator.createServices()) {
				return coordinator.runNode();
		} else {
			 return coordinator.shutdownAndExit();
		}
	} catch (std::exception &e) {
		return coordinator.shutdownAndExit(e);
	}
}
