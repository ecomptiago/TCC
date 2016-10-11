/*
 * pathPlanner.cpp
 *
 *  Created on: Dec 15, 2015
 *      Author: tiago
 */

#include "path_planner/PathPlanner.h"

//Constructors
PathPlanner::PathPlanner(int argc, char **argv, int cellArea, int mapWidth, int mapHeight,
	float angleTolerance, double wakeUpTime) : BaseRosNode(argc, argv, "Path_planner"){
		this->occupancyGrid.info.resolution = cellArea;
		this->occupancyGrid.info.width = mapWidth;
		this->occupancyGrid.info.height = mapHeight;
		this->occupancyGrid.header.frame_id = "LaserScannerBody_2D";
		for(int i = 0; i < (mapWidth * mapHeight) / cellArea; i++) {
			this->occupancyGrid.data.insert(this->occupancyGrid.data.begin(),unknownCell);
		}
		this->angleTolerance = angleTolerance;
		this->wakeUpTime = wakeUpTime;
		this->aStar = AStar();
		this->neuralGrid.data.resize(49);
}

//Methods
int PathPlanner::runNode() {
	ROS_INFO("Running node");

	path_planner::simRosGetObjectChild simRosGetObjectChild;
	bool firstLoop = true;
	int index = 0;


	//Getting origin coordinates
	if(VRepUtils::getObjectHandle(floorHandle,nodeHandler,signalObjectMap)) {
		geometry_msgs::PoseStamped origin;
		do{
			simRosGetObjectChild.request.handle = signalObjectMap[floorHandle];
			simRosGetObjectChild.request.index = index;
			serviceClientsMap[getObjectChildService].call(simRosGetObjectChild);
			int32_t childHandle = simRosGetObjectChild.response.childHandle;
			if (childHandle != responseError) {
				common::simRosGetObjectPose simRosGetObjectPose;
				if(VRepUtils::getObjectPose(simRosGetObjectChild.response.childHandle,
					nodeHandler,simRosGetObjectPose)) {
						if(firstLoop || (
							simRosGetObjectPose.response.pose.pose.position.x < origin.pose.position.x &&
							simRosGetObjectPose.response.pose.pose.position.y < origin.pose.position.y)) {
								origin = simRosGetObjectPose.response.pose;
								firstLoop = false;
						}
				} else {
					ROS_ERROR("Could not get pose of a children from object %s",floorHandle);
					return shutdownAndExit();
				}
			}
			index++;
		} while(simRosGetObjectChild.response.childHandle != responseError);
		occupancyGrid.info.origin.orientation = origin.pose.orientation;
		occupancyGrid.info.origin.position.x = origin.pose.position.x - 2.5;
		occupancyGrid.info.origin.position.y = origin.pose.position.y - 2.5;
	} else {
		ROS_ERROR("Could not found handle for object %s",floorHandle);
		return shutdownAndExit();
	}

//	occupancyGrid.data[20] = occupiedCell;
//	occupancyGrid.data[21] = occupiedCell;
//	occupancyGrid.data[22] = occupiedCell;
//	occupancyGrid.data[23] = occupiedCell;
//	occupancyGrid.data[24] = occupiedCell;
//
//	occupancyGrid.data[43] = occupiedCell;
//	occupancyGrid.data[44] = occupiedCell;
//	occupancyGrid.data[45] = occupiedCell;
//	occupancyGrid.data[46] = occupiedCell;
//	occupancyGrid.data[47] = occupiedCell;
//
//	occupancyGrid.data[71] = occupiedCell;
//	occupancyGrid.data[72] = occupiedCell;
//	occupancyGrid.data[73] = occupiedCell;
//	occupancyGrid.data[74] = occupiedCell;
//	occupancyGrid.data[75] = occupiedCell;

	ros::Rate rate(1/wakeUpTime);

 	while(ros::ok()) {
		sleepAndSpin(rate);
		double angle = OdometryUtils::getAngleFromQuaternation(
 	 		tf::Quaternion(0,0,
 			robotPose.pose.orientation.z,
 			robotPose.pose.orientation.w),false);
 		if(NumericUtils::isFirstLess<float>(angle,0.0)) {
 			angle = angle + 360;
 		}

 		ROS_DEBUG("Angle %f",angle);
 		if(NumericUtils::isFirstGreaterEqual<float>(angle,80) &&
 			NumericUtils::isFirstLessEqual<float>(angle, 100)) {
 				angle = 90;
 		} else if(NumericUtils::isFirstGreaterEqual<float>(angle,170) &&
 			NumericUtils::isFirstLessEqual<float>(angle, 190)) {
 				angle = 180;
 		} else if(NumericUtils::isFirstGreaterEqual<float>(angle,260) &&
 	 		NumericUtils::isFirstLessEqual<float>(angle, 280)) {
 			 angle = 270;
 		} else if((NumericUtils::isFirstGreaterEqual<float>(angle,0) &&
 			NumericUtils::isFirstLessEqual<float>(angle, 10)) ||
 			(NumericUtils::isFirstGreaterEqual<float>(angle,350) &&
 			NumericUtils::isFirstLessEqual<float>(angle, 360))) {
 				angle = 0;
 		}

 		angle = (angle * M_PI) / 180;

 		int i = 0;

 		for(double u = 1; u < 8; u++ ) {
 			for(double v = 3; v > -4; v--) {
 				float x = (u * cos(angle)) - (v * sin(angle));
 				float y = (u * sin(angle)) + (v * cos(angle));
 				x = x + robotPose.pose.position.x;
 				y = y + robotPose.pose.position.y;
 				float cellValue;
 				common::Position position;
 				position.x = x;
 				position.y = y;
 				int cellPosition =
 					PathPlannerUtils::getDataVectorPosition(occupancyGrid,position);
// 				ROS_DEBUG("Position.x %f , Position.y %f , cellPosition %d", x, y, cellPosition);
 				if(cellPosition != -1) {
					if(NumericUtils::isFirstLess<float>(neuralGrid.data[i],0.0)) {
						cellValue = neuralGrid.data[i] * -1;
					} else {
						cellValue = neuralGrid.data[i];
					}
					if(NumericUtils::isFirstLessEqual<float>(cellValue, 0.5)) {
						occupancyGrid.data[cellPosition] = freeCell;
					} else {
						occupancyGrid.data[cellPosition] = occupiedCell;
					}
 				}
				i++;
 			}
 		}

		publisherMap[mapTopic].publish(occupancyGrid);
	}

	return shutdownAndExit();
}

bool PathPlanner::subscribeToTopics() {
	return addSubscribedTopic<const geometry_msgs::PoseStamped::ConstPtr&,PathPlanner>(nodeHandler,poseTopic,
		&PathPlanner::receivedRobotPose,this) &&

		addSubscribedTopic<const std_msgs::Float32MultiArray::ConstPtr&,PathPlanner>(nodeHandler,neuralGridTopic,
			&PathPlanner::receivedNeuralGrid,this);
}

bool PathPlanner::createServices() {
	ROS_INFO("Creating services");
	if(createServiceClients()) {
		return createServiceServers();
	} else {
		return false;
	}
}

bool PathPlanner::createServiceClients() {
	return addServiceClient<path_planner::simRosGetObjectGroupData>(nodeHandler, getObjectGroupDataService) &&
		   addServiceClient<path_planner::simRosGetObjectFloatParameter>(nodeHandler, getObjectFloatParameterService) &&
		   addServiceClient<path_planner::simRosGetObjectChild>(nodeHandler,getObjectChildService);
}

bool PathPlanner::createServiceServers() {
	return addServiceServer<common::pathToTarget::Request&,
		common::pathToTarget::Response&, PathPlanner>(nodeHandler,bestPathService, &PathPlanner::bestPath,this) &&

		addServiceServer<common::cellGridPosition::Request&,
		common::cellGridPosition::Response&, PathPlanner>(nodeHandler,cellGridPositionService, &PathPlanner::cellGridPosition,this);
}

bool PathPlanner::createPublishers() {
	return addPublisherClient<nav_msgs::OccupancyGrid>(nodeHandler,mapTopic,false);
}

//Callback
void PathPlanner::receivedRobotPose(const geometry_msgs::PoseStamped::ConstPtr& robotPose){
	this->robotPose.header = robotPose->header;
	this->robotPose.pose = robotPose->pose;
}

void PathPlanner::receivedNeuralGrid(const std_msgs::Float32MultiArray::ConstPtr& neuralGrid) {
	for(int i = 0 ; i < neuralGrid->data.capacity(); i++) {
		this->neuralGrid.data[i] = neuralGrid->data[i];
	}
}

bool PathPlanner::bestPath(common::pathToTarget::Request  &req,
	common::pathToTarget::Response &res) {
		if (VRepUtils::getObjectHandle(pionnerHandle,nodeHandler,signalObjectMap)) {
			common::simRosGetObjectPose simRosGetObjectPose;
			aStar.setOccupancyGrid(occupancyGrid);
			if(VRepUtils::getObjectPose(signalObjectMap[pionnerHandle], nodeHandler,simRosGetObjectPose)) {
				common::Position targetPosition;
				targetPosition.x = req.x;
				targetPosition.y = req.y;

				common::Position initialPosition;
				initialPosition.x = simRosGetObjectPose.response.pose.pose.position.x;
				initialPosition.y = simRosGetObjectPose.response.pose.pose.position.y;

				if(aStar.findPathToGoal(initialPosition ,targetPosition)) {
					std::vector<AStarGridCell> path;
					aStar.reconstructPath(path, targetPosition,initialPosition);

					int charsWrote = 0;
					char buffer [occupancyGrid.data.size() * 6];
					res.path.resize(path.size());

					for(int i = 0; i < path.size();i++) {
						int cellPosition = path[i].cellGridPosition;
						res.path[i] = cellPosition;
						charsWrote += sprintf(buffer + charsWrote,
							" %d,",cellPosition);
					}
					ROS_DEBUG("Optimized path: [%s]",buffer);
					ROS_INFO("Found path to (%f,%f)",req.x,req.y);
				} else {
					int targetCellPosition =
						PathPlannerUtils::getDataVectorPosition(occupancyGrid, targetPosition);
					if(occupancyGrid.data[targetCellPosition] == unknownCell) {
						res.path.clear();
						ROS_INFO("Could not find path to (%f,%f)",req.x,req.y);
					} else {
						res.path.resize(1);
						res.path[0] = -2;
					}
				}
			} else {
				res.path[0] = -3;
				res.path.clear();
				ROS_INFO("Could not get robot position");
			}
		} else {
			res.path[0] = -3;
			res.path.clear();
			ROS_INFO("Could not get robot v-rep handler");
		}
		return true;
}

bool PathPlanner::cellGridPosition(common::cellGridPosition::Request  &req,
	common::cellGridPosition::Response &res) {
		common::Position position;
		PathPlannerUtils::getCoordinatesFromDataVectorPosition(occupancyGrid,position,req.gridPosition);
		res.x = position.x;
		res.y = position.y;
		return true;
}

//Main
int main(int argc, char **argv) {
	PathPlanner pathPlanner(argc,argv,1,10,10,10,0.5);
	try{
		if(pathPlanner.createServices() &&
			pathPlanner.subscribeToTopics() &&
			pathPlanner.createPublishers()) {
				return pathPlanner.runNode();
		} else {
			return pathPlanner.shutdownAndExit();
		}
	} catch (std::exception &e) {
		return pathPlanner.shutdownAndExit(e);
	}
}
