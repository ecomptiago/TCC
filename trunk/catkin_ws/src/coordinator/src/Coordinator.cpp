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
		recalculatePath = false;
		pathPosition = -1;
		proportionalError.data = -1;

		float originX = -6.15;
		float originY = 0.72;
		common::Position targetPosition;

//		targetPosition.x = originX + 0.5;
//		targetPosition.y = originY + 0.5;
//		targetPositions.push_back(targetPosition);
//
//		targetPosition.x = originX + 0.5 + 9;
//		targetPosition.y = originY + 0.5;
//		targetPositions.push_back(targetPosition);
//
//		targetPosition.x = originX + 0.5;
//		targetPosition.y = originY + 0.5;
//		targetPositions.push_back(targetPosition);
//
//		targetPosition.x = originX + 0.5;
//		targetPosition.y = originY + 0.5 + 9;
//		targetPositions.push_back(targetPosition);

//		targetPosition.x = originX + 1.5;
//		targetPosition.y = originY + 1.5;
//		targetPositions.push_back(targetPosition);

		targetPosition.x = originX + 0.5 + 9;
		targetPosition.y = originY + 0.5 + 9;
		targetPositions.push_back(targetPosition);

//		targetPosition.x = originX + 0.5;
//		targetPosition.y = originY + 0.5;
//		targetPositions.push_back(targetPosition);

}

//Methods
int Coordinator::runNode() {
	ROS_INFO("Running node");
	ros::Rate rate(1/0.5);
	common::pathToTarget pathToTarget;
	geometry_msgs::Twist stop;
	stop.angular.x = 0;
	stop.angular.y = 0;
	stop.angular.z = 0;
	stop.linear.x = 0;
	stop.linear.y = 0;
	stop.linear.z = 0;
	while(ros::ok()) {
		sleepAndSpin(rate);
		if(laserValues.capacity() != 0) {
			float smallestLaserReading = *std::min_element(laserValues.begin(),
				laserValues.end());
			pathToTarget.request.x = targetPositions.back().x;
			pathToTarget.request.y = targetPositions.back().y;
			int cellGridIndex;
			if(!reachedFinalGoal) {
				if(!triedToFindPath ||
					(recalculatePath && NumericUtils::isFirstGreater<float>(smallestLaserReading, 0.5))) {
						ROS_DEBUG("Calling service to get best path to x:%f y:%f ",
							pathToTarget.request.x,pathToTarget.request.y);
						serviceClientsMap[bestPathService].call(pathToTarget);
						pathPosition = 0;
						triedToFindPath = true;
						recalculatePath = false;
				} else {
					common::Position position;
					if(pathToTarget.response.path.size() == 0) {
						ROS_DEBUG("Could not find path to x:%f y:%f",
							pathToTarget.request.x,pathToTarget.request.y);
						position.x = pathToTarget.request.x;
						position.y = pathToTarget.request.y;
						publisherMap[targetPositionTopic].publish(position);
						cellGridIndex = GridUtils::getDataVectorPosition(occupancyGrid,position);
						for(int i = 0; i < 20; i++) {
							sleepAndSpin(50);
						}
					} else if(pathToTarget.response.path.size() > 1){
						ROS_DEBUG("Found a path to x:%f y:%f",
							pathToTarget.request.x,pathToTarget.request.y);
						int charsWrote = 0;
						char buffer [pathToTarget.response.path.size() * 6];

						for(int i = 0; i < pathToTarget.response.path.size();i++) {
							charsWrote += sprintf(buffer + charsWrote,
								" %d,",pathToTarget.response.path[i]);
						}
						ROS_DEBUG("Optimized path: [%s]",buffer);

						common::Position position = cellGridPosition(pathToTarget.response.path[pathPosition]);
						ROS_DEBUG("Going to cell %d",pathToTarget.response.path[pathPosition]);
						publisherMap[targetPositionTopic].publish(position);
						cellGridIndex = GridUtils::getDataVectorPosition(occupancyGrid,position);
						for(int i = 0; i < 20; i++) {
							sleepAndSpin(50);
						}
					} else if(pathToTarget.response.path.size() == 1){
						if(pathToTarget.response.path[0] == -3) {
							ROS_ERROR("Could not retrieve data from V-Rep");
							return shutdownAndExit();
						} else {
							ROS_DEBUG("Cell is occupied");
							publisherMap[cmdVelTopic].publish(stop);
							ROS_DEBUG("Setting velocity liner 0 and angular 0");
							reachedFinalGoal = true;
							triedToFindPath = false;
						}
					}

					if(NumericUtils::isFirstGreaterEqual<float>(proportionalError.data,0.75) && !reachedFinalGoal
						&& occupancyGrid.data[cellGridIndex] != 100) {
						ROS_DEBUG("The smallest element is %f",smallestLaserReading);
						ROS_DEBUG("Proportional error %f",proportionalError.data);
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
						if((pathToTarget.response.path.size() == 0 &&
							NumericUtils::isFirstLessEqual<float>(proportionalError.data,0.75) &&
							NumericUtils::isFirstGreaterEqual<float>(proportionalError.data,0))) {
								publisherMap[cmdVelTopic].publish(stop);
								ROS_DEBUG("Setting velocity liner 0 and angular 0");
								reachedFinalGoal = true;
								triedToFindPath = false;
						} else if(pathToTarget.response.path.size() > 1){
							ROS_DEBUG("pathPosition %d pathToTarget.response.path.size() %lu",
								pathPosition,pathToTarget.response.path.size());
							if(pathPosition == pathToTarget.response.path.size() + 1) {
								publisherMap[cmdVelTopic].publish(stop);
								ROS_DEBUG("Setting velocity liner 0 and angular 0");
								reachedFinalGoal = true;
								triedToFindPath = false;
							} else {
								pathPosition++;
								ROS_DEBUG("Going to %d cell of the path",pathPosition);
							}
						}
					}
				}
			} else {
				if(targetPositions.size() != 0) {
					reachedFinalGoal = false;
					targetPositions.pop_back();
				}
			}

		}
	}
	return shutdownAndExit();
}

const common::Position Coordinator::cellGridPosition(int cellGrid) {
	common::Position position;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid,position,cellGrid);
	return position;
}

bool Coordinator::subscribeToTopics() {
	ROS_INFO("Subscribing to topics");
	return addSubscribedTopic<const sensor_msgs::LaserScan::ConstPtr&, Coordinator>(nodeHandler,laserTopic,
		&Coordinator::receivedLaserValues,this) &&

		addSubscribedTopic<const geometry_msgs::Twist::ConstPtr&,Coordinator>(nodeHandler,velTopic,
			&Coordinator::receivedProportionalControlerVelocity,this) &&

		addSubscribedTopic<const std_msgs::Float32::ConstPtr&,Coordinator>(nodeHandler,errorTopic,
			&Coordinator::receivedProportionalControlerError,this) &&

		addSubscribedTopic<const std_msgs::Float32::ConstPtr&,Coordinator>(nodeHandler,turnAngleTopic,
			&Coordinator::receivedFuzzyTurnAngle,this);

		addSubscribedTopic<const nav_msgs::OccupancyGrid::ConstPtr&,Coordinator>(nodeHandler,occupancyGridTopic,
			&Coordinator::receivedOccupancyGrid,this);

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
	return addServiceClient<common::pathToTarget>(nodeHandler,bestPathService);
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

void Coordinator::receivedOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid) {
	this->occupancyGrid.info = occupancyGrid->info;
	this->occupancyGrid.header = occupancyGrid->header;
	this->occupancyGrid.data = occupancyGrid->data;
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
