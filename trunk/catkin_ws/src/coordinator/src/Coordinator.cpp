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
}

//Methods
int Coordinator::runNode() {
	ROS_INFO("Running node");
	bool firstTime = true;
	bool comingBack = true;
	bool updateWorld = true;
	int count = 0;
	float fuzzyTresholdToAct = 0.4;
	float proportionalErrorToAchieve = 0.8;
	float constantLinearFuzzy = 1.5;
	float constantAngularFuzzy = 0.05;
	float angleErrorMargin = 2.5;
	common::pathToTarget pathToTarget;
	common::Position targets[20];
	geometry_msgs::Twist stop;
	stop.angular.x = 0;
	stop.angular.y = 0;
	stop.angular.z = 0;
	stop.linear.x = 0;
	stop.linear.y = 0;
	stop.linear.z = 0;
	while(ros::ok()) {
		sleepAndSpin(500);
		float smallestLaserReading = *std::min_element(laserValues.begin(),
			laserValues.end());
		if(count < 10 && occupancyGrid.info.height != 0 && laserValues.capacity() != 0) {
			if(firstTime) {
				ROS_INFO("Mapeando o ambiente");
				createSlamPath(targets);
				firstTime = false;
			}
			ROS_INFO("Indo para a posicao x:%f y:%f", targets[count].x, targets[count].y);
			publisherMap[targetPositionTopic].publish(targets[count]);
			sleepAndSpin(3000);
			while(NumericUtils::isFirstGreaterEqual<float>(proportionalError.data,proportionalErrorToAchieve)) {
				sleepAndSpin(500);
				smallestLaserReading = *std::min_element(laserValues.begin(),
					laserValues.end());
				ROS_DEBUG("smallestLaserReading %f",smallestLaserReading);
				if(NumericUtils::isFirstGreater<float>(smallestLaserReading, fuzzyTresholdToAct)) {
					ROS_INFO("Usando controlador cinematico");
					publisherMap[cmdVelTopic].publish(proportionalVelocity);
				} else {
					ROS_INFO("Usando controlador fuzzy");
					geometry_msgs::Twist move;
					move.linear.x = smallestLaserReading * constantLinearFuzzy;
					move.angular.z = constantAngularFuzzy * fuzzyTurnAngle.data;
					ROS_DEBUG("Setting fuzzy velocity linear %f and angular %f",move.linear.x,move.angular.z);
					publisherMap[cmdVelTopic].publish(move);
				}
			}
			publisherMap[cmdVelTopic].publish(stop);
			sleepAndSpin(3000);

//			geometry_msgs::Twist rotate;
//			rotate.angular.x = 0;
//			rotate.angular.y = 0;
//			rotate.angular.z = 0.2;
//			rotate.linear.x = 0;
//			rotate.linear.y = 0;
//			rotate.linear.z = 0;
//			publisherMap[cmdVelTopic].publish(rotate);

//			double angle = OdometryUtils::getAngleFromQuaternation(
//				tf::Quaternion(0,0,
//				robotPose.pose.orientation.z,
//				robotPose.pose.orientation.w),false);
//			double desiredAngle = 90 * ((count + 1) % 4);
//
//			ROS_DEBUG("desiredAngle %f", desiredAngle);
//
//			while(NumericUtils::isFirstGreaterEqual<float>(angle,desiredAngle - angleErrorMargin) &&
//				NumericUtils::isFirstLessEqual<float>(angle,desiredAngle + angleErrorMargin)) {
//					sleepAndSpin(500);
//					angle = OdometryUtils::getAngleFromQuaternation(
//						tf::Quaternion(0,0,
//						robotPose.pose.orientation.z,
//						robotPose.pose.orientation.w),false);
//			}
			count++;
		}
		else if(count >= 10 && count < 20 && occupancyGrid.info.height != 0 && laserValues.capacity() != 0){
//			if(comingBack) {
//				double angle = OdometryUtils::getAngleFromQuaternation(
//					tf::Quaternion(0,0,
//					robotPose.pose.orientation.z,
//					robotPose.pose.orientation.w),false);
//				double desiredAngle = 270;
//
//				ROS_DEBUG("desiredAngle %f", desiredAngle);
//
//				while(NumericUtils::isFirstGreaterEqual<float>(angle,desiredAngle - angleErrorMargin) &&
//					 NumericUtils::isFirstLessEqual<float>(angle,desiredAngle +angleErrorMargin)) {
//						sleepAndSpin(250);
//						angle = OdometryUtils::getAngleFromQuaternation(
//							tf::Quaternion(0,0,
//							robotPose.pose.orientation.z,
//							robotPose.pose.orientation.w),false);
//				}
//				comingBack = false;
//			}
			publisherMap[targetPositionTopic].publish(targets[count]);
			ROS_INFO("Indo para a posicao x:%f y:%f", targets[count].x, targets[count].y);
			sleepAndSpin(3000);
			while(NumericUtils::isFirstGreaterEqual<float>(proportionalError.data,proportionalErrorToAchieve)) {
				sleepAndSpin(500);
				smallestLaserReading = *std::min_element(laserValues.begin(),
						laserValues.end());
				ROS_DEBUG("smallestLaserReading %f",smallestLaserReading);
				if(NumericUtils::isFirstGreater<float>(smallestLaserReading, fuzzyTresholdToAct)) {
					ROS_INFO("Usando controlador cinematico");
					publisherMap[cmdVelTopic].publish(proportionalVelocity);
				} else {
					ROS_INFO("Usando controlador fuzzy");
					geometry_msgs::Twist move;
					move.linear.x = smallestLaserReading * constantLinearFuzzy;
					move.angular.z = constantAngularFuzzy * fuzzyTurnAngle.data;
					ROS_DEBUG("Setting fuzzy velocity linear %f and angular %f",move.linear.x,move.angular.z);
					publisherMap[cmdVelTopic].publish(move);
				}
			}
			publisherMap[cmdVelTopic].publish(stop);
			sleepAndSpin(3000);

//			geometry_msgs::Twist rotate;
//			rotate.angular.x = 0;
//			rotate.angular.y = 0;
//			rotate.angular.z = -0.2;
//			rotate.linear.x = 0;
//			rotate.linear.y = 0;
//			rotate.linear.z = 0;
//			publisherMap[cmdVelTopic].publish(rotate);

//			double angle = OdometryUtils::getAngleFromQuaternation(
//				tf::Quaternion(0,0,
//				robotPose.pose.orientation.z,
//				robotPose.pose.orientation.w),false);
//			double desiredAngle = -90 * ((count + 1) % 4);
//
//			ROS_DEBUG("desiredAngle %f", desiredAngle);
//
//			while(NumericUtils::isFirstGreaterEqual<float>(angle,desiredAngle - angleErrorMargin) &&
//				  NumericUtils::isFirstLessEqual<float>(angle,desiredAngle + angleErrorMargin)) {
//					sleepAndSpin(500);
//					angle = OdometryUtils::getAngleFromQuaternation(
//						tf::Quaternion(0,0,
//						robotPose.pose.orientation.z,
//						robotPose.pose.orientation.w),false);
//			}
			count++;
		} else if(occupancyGrid.info.height != 0 && laserValues.capacity() != 0) {
			if(updateWorld) {
				std_msgs::Bool updateWorldTemp;
				updateWorldTemp.data = updateWorld;
				publisherMap[updateWorldTopic].publish(updateWorldTemp);
				sleepAndSpin(3000);
				updateWorld = false;
				ROS_INFO("Navegando por um ambiente mapeado");
			}
			int cellGridIndex;
			common::pathToTarget pathTotarget;
			pathToTarget.request.x = targets[17].x;
			pathToTarget.request.y = targets[17].y;
			if(!reachedFinalGoal && pathToTarget.request.x != 0 && pathToTarget.request.y != 0) {
				if(!triedToFindPath ||
				   (recalculatePath && NumericUtils::isFirstGreater<float>(smallestLaserReading, fuzzyTresholdToAct))) {
						ROS_INFO("Chamando servico para obter melhor caminho ate a posicao x:%f y:%f",
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
						sleepAndSpin(3000);
					} else if(pathToTarget.response.path.size() > 1){
						ROS_DEBUG("Found a path to x:%f y:%f",
							pathToTarget.request.x,pathToTarget.request.y);
						int charsWrote = 0;
						char buffer [pathToTarget.response.path.size() * 6];
			
						for(int i = 0; i < pathToTarget.response.path.size();i++) {
							charsWrote += sprintf(buffer + charsWrote,
								" %d,",pathToTarget.response.path[i]);
						}
						ROS_INFO("Achou o caminho [%s]",buffer);
			
						common::Position position = cellGridPosition(pathToTarget.response.path[pathPosition]);
						ROS_INFO("Indo para a celula %d",pathToTarget.response.path[pathPosition]);
						publisherMap[targetPositionTopic].publish(position);
						sleepAndSpin(3000);
					} else if(pathToTarget.response.path.size() == 1){
						if(pathToTarget.response.path[0] == -3) {
							ROS_ERROR("Could not retrieve data from V-Rep");
							return shutdownAndExit();
						} else {
							ROS_DEBUG("Cell is occupied");
							publisherMap[cmdVelTopic].publish(stop);
							sleepAndSpin(3000);
							ROS_DEBUG("Setting velocity liner 0 and angular 0");
							reachedFinalGoal = true;
							triedToFindPath = false;
						}
					}
				}
			
				if(NumericUtils::isFirstGreaterEqual<float>(proportionalError.data,proportionalErrorToAchieve) && !reachedFinalGoal) {
					ROS_DEBUG("The smallest element is %f",smallestLaserReading);
					ROS_DEBUG("Proportional error %f",proportionalError.data);
					if(NumericUtils::isFirstLessEqual<float>(smallestLaserReading, fuzzyTresholdToAct)) {
						ROS_INFO("Usando controlador fuzzy");
						geometry_msgs::Twist move;
						move.linear.x = constantLinearFuzzy * smallestLaserReading;
						ROS_DEBUG("fuzzyTurnAngle %f",fuzzyTurnAngle.data);
						move.angular.z = constantAngularFuzzy * fuzzyTurnAngle.data ;
						publisherMap[cmdVelTopic].publish(move);
						ROS_DEBUG("Setting velocity liner %f and angular %f",move.linear.x,move.angular.z);
						recalculatePath = true;
					} else {
						ROS_INFO("Usando controlador cinematico");
						publisherMap[cmdVelTopic].publish(proportionalVelocity);
						ROS_DEBUG("Setting velocity liner %f and angular %f",
							proportionalVelocity.linear.x,proportionalVelocity.angular.z);
					}
				} else {
					if((pathToTarget.response.path.size() == 0 &&
						NumericUtils::isFirstLessEqual<float>(proportionalError.data,proportionalErrorToAchieve) &&
						NumericUtils::isFirstGreaterEqual<float>(proportionalError.data,0))) {
							publisherMap[cmdVelTopic].publish(stop);
							ROS_DEBUG("Setting velocity liner 0 and angular 0");
							reachedFinalGoal = true;
							triedToFindPath = false;
					} else if(pathToTarget.response.path.size() > 1){
						ROS_DEBUG("pathPosition %d pathToTarget.response.path.size() %lu",
							pathPosition,pathToTarget.response.path.size());
						if(pathPosition == pathToTarget.response.path.size()) {
							publisherMap[cmdVelTopic].publish(stop);
							sleepAndSpin(3000);
							ROS_DEBUG("Setting velocity liner 0 and angular 0");
							reachedFinalGoal = true;
							triedToFindPath = false;
						} else {
							pathPosition++;
						}
					}
				}
			}
		}
	}
	return shutdownAndExit();
}

void Coordinator::createSlamPath(common::Position targets[20]) {
	common::Position pos;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid, pos, 9);
	targets[0] = pos;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid, pos, 99);
	targets[1] = pos;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid, pos, 90);
	targets[2] = pos;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid, pos, 20);
	targets[3] = pos;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid, pos, 27);
	targets[4] = pos;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid, pos, 77);
	targets[5] = pos;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid, pos, 72);
	targets[6] = pos;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid, pos, 42);
	targets[7] = pos;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid, pos, 45);
	targets[8] = pos;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid, pos, 65);
	targets[9] = pos;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid, pos, 45);
	targets[10] = pos;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid, pos, 42);
	targets[11] = pos;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid, pos, 72);
	targets[12] = pos;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid, pos, 77);
	targets[13] = pos;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid, pos, 27);
	targets[14] = pos;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid, pos, 20);
	targets[15] = pos;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid, pos, 90);
	targets[16] = pos;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid, pos, 99);
	targets[17] = pos;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid, pos, 9);
	targets[18] = pos;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid, pos, 0);
	targets[19] = pos;
}

const common::Position Coordinator::cellGridPosition(int cellGrid) {
	common::Position position;
	GridUtils::getCoordinatesFromDataVectorPosition(occupancyGrid,position,cellGrid);
	return position;
}

bool Coordinator::subscribeToTopics() {
	ROS_INFO("Subscribing to topics");
	return addSubscribedTopic<const common::Position::ConstPtr&,Coordinator>(nodeHandler,targetTopic,
		&Coordinator::receiveTarget,this) &&

		addSubscribedTopic<const geometry_msgs::PoseStamped::ConstPtr&, Coordinator>(nodeHandler,poseTopic,
			&Coordinator::receivedRobotPose,this) &&

		addSubscribedTopic<const sensor_msgs::LaserScan::ConstPtr&, Coordinator>(nodeHandler,laserTopic,
			&Coordinator::receivedLaserValues,this) &&

		addSubscribedTopic<const geometry_msgs::Twist::ConstPtr&,Coordinator>(nodeHandler,velTopic,
			&Coordinator::receivedProportionalControlerVelocity,this) &&

		addSubscribedTopic<const std_msgs::Float32::ConstPtr&,Coordinator>(nodeHandler,errorTopic,
			&Coordinator::receivedProportionalControlerError,this) &&

		addSubscribedTopic<const std_msgs::Float32::ConstPtr&,Coordinator>(nodeHandler,turnAngleTopic,
			&Coordinator::receivedFuzzyTurnAngle,this) &&

		addSubscribedTopic<const nav_msgs::OccupancyGrid::ConstPtr&,Coordinator>(nodeHandler,occupancyGridTopic,
			&Coordinator::receivedOccupancyGrid,this);

}

bool Coordinator::createPublishers() {
	ROS_INFO("Creating publishers");
	return addPublisherClient<common::Position>(
		nodeHandler, targetPositionTopic, false) &&

		addPublisherClient<geometry_msgs::Twist>(
			nodeHandler,cmdVelTopic,false) &&

		addPublisherClient<std_msgs::Bool>(
			nodeHandler, updateWorldTopic, false)	;
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

void Coordinator::receiveTarget(const common::Position::ConstPtr& target){
	common::Position pos;
	pos.x = robotPose.pose.position.x + target->x;
	pos.y = robotPose.pose.position.y + target->y;
	reachedFinalGoal = false;
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
