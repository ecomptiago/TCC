/*
 * pathPlanner.cpp
 *
 *  Created on: Dec 15, 2015
 *      Author: tiago
 */

#include "path_planner/PathPlanner.h"

//Constructors
PathPlanner::PathPlanner(int argc, char **argv, int cellArea, int mapWidth, int mapHeight) :
	BaseRosNode(argc, argv, "Path_planner"){
		occupancyGrid.info.resolution = cellArea;
		occupancyGrid.info.width = mapWidth;
		occupancyGrid.info.height = mapHeight;
}

//Methods
int PathPlanner::runNode() {
	ROS_INFO("Running node");

	path_planner::simRosGetObjectChild simRosGetObjectChild;
	bool firstLoop = true;
	int index = 0;

	//Getting origin coordinates
	if(VRepUtils::getObjectHandle(floorHandle,nodeHandler,signalObjectMap)) {
		common::Position position;
		do{
			simRosGetObjectChild.request.handle = signalObjectMap[floorHandle];
			simRosGetObjectChild.request.index = index;
			serviceClientsMap[getObjectChildService].call(simRosGetObjectChild);
			int32_t childHandle = simRosGetObjectChild.response.childHandle;
			if (childHandle != responseError) {
				common::simRosGetObjectPose simRosGetObjectPose;
				if(VRepUtils::getObjectPose(simRosGetObjectChild.response.childHandle,
					nodeHandler,simRosGetObjectPose)) {
						common::Position objectPosition;
						if(getMinimumXYObjectCoordinate(childHandle,simRosGetObjectPose, objectPosition)) {
							if(firstLoop || (objectPosition.x < position.x && objectPosition.y < position.y)) {
								position = objectPosition;
								firstLoop = false;
							}
						} else {
							ROS_ERROR("Could not get minimum x y of a children from object %s",floorHandle);
							shutdownAndExit();
						}
				} else {
					ROS_ERROR("Could not get pose of a children from object %s",floorHandle);
					shutdownAndExit();
				}
			}
			index++;
		} while(simRosGetObjectChild.response.childHandle != responseError);
		geometry_msgs::Pose pose;
		pose.position.x = position.x;
		pose.position.y = position.y;
		occupancyGrid.info.origin = pose;
	} else {
		ROS_ERROR("Could not found handle for object %s",floorHandle);
		shutdownAndExit();
	}

	if(VRepUtils::getObjectHandle(cuboidHandle,nodeHandler,signalObjectMap)) {
		index = 0;
		do{
			simRosGetObjectChild.request.handle = signalObjectMap[cuboidHandle];
			simRosGetObjectChild.request.index = index;
			serviceClientsMap[getObjectChildService].call(simRosGetObjectChild);
			int32_t childHandle = simRosGetObjectChild.response.childHandle;
			if (childHandle != responseError) {
				common::simRosGetObjectPose simRosGetObjectPose;
				if(VRepUtils::getObjectPose(simRosGetObjectChild.response.childHandle,
					nodeHandler,simRosGetObjectPose)) {

				} else {
					ROS_ERROR("Could not get pose of a children from object %s",cuboidHandle);
					shutdownAndExit();
				}
			}
			index++;
		} while(simRosGetObjectChild.response.childHandle != responseError);
	} else {
		ROS_ERROR("Could not found handle for object %s",cuboidHandle);
		shutdownAndExit();
	}

	ros::spin();
	shutdownAndExit();
}

bool PathPlanner::getMinimumXYObjectCoordinate(int32_t objecthandle,
	common::simRosGetObjectPose &simRosGetObjectPose,common::Position &position){
	path_planner::ObjectInfo objectInfo;
	if(getObjectWidthHeight(objecthandle,objectInfo)) {
		position.x = simRosGetObjectPose.response.pose.pose.position.x - (objectInfo.width / 2);
		position.y = simRosGetObjectPose.response.pose.pose.position.y - (objectInfo.height / 2);
		return true;
	} else {
		return false;
	}
}

bool PathPlanner::getObjectWidthHeight(int32_t objectHandle,
	path_planner::ObjectInfo &objectInfo) {
		path_planner::simRosGetObjectFloatParameter simRosGetObjectFloatParameter;
		callGetFloatParameterService(objectHandle,sim_objfloatparam_modelbbox_min_x,simRosGetObjectFloatParameter);
		if(simRosGetObjectFloatParameter.response.result == responseError) {
			return false;
		} else {
			float minX = simRosGetObjectFloatParameter.response.parameterValue;
			callGetFloatParameterService(objectHandle,sim_objfloatparam_modelbbox_max_x,simRosGetObjectFloatParameter);
			if(simRosGetObjectFloatParameter.response.result == responseError) {
				return false;
			} else {
				objectInfo.width = simRosGetObjectFloatParameter.response.parameterValue - minX;
			}
		}
		callGetFloatParameterService(objectHandle,sim_objfloatparam_modelbbox_min_y,simRosGetObjectFloatParameter);
		if(simRosGetObjectFloatParameter.response.result == responseError) {
			return false;
		} else {
			float minY = simRosGetObjectFloatParameter.response.parameterValue;
			callGetFloatParameterService(objectHandle,sim_objfloatparam_modelbbox_max_x,simRosGetObjectFloatParameter);
			if(simRosGetObjectFloatParameter.response.result == responseError) {
				return false;
			} else {
				objectInfo.height = simRosGetObjectFloatParameter.response.parameterValue - minY;
				return true;
			}
		}
}

void PathPlanner::callGetFloatParameterService(int32_t objectHandle,
	int32_t parameterID,
	path_planner::simRosGetObjectFloatParameter &simRosGetObjectFloatParameter) {
		simRosGetObjectFloatParameter.request.handle = objectHandle;
		simRosGetObjectFloatParameter.request.parameterID =
				parameterID;
		serviceClientsMap[getObjectFloatParameterService].call(
			simRosGetObjectFloatParameter);
}

bool PathPlanner::subscribeToTopics() {
	return true;
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
	return true;
}

bool PathPlanner::createPublishers() {
	return true;
}

//Callback

//Main
int main(int argc, char **argv) {
	PathPlanner pathPlanner(argc,argv,1,10,10);
	try{
		if(pathPlanner.createServices() &&
			pathPlanner.subscribeToTopics() &&
			pathPlanner.createPublishers()) {
				return pathPlanner.runNode();
		} else {
			pathPlanner.shutdownAndExit();
		}
	} catch (std::exception &e) {
		pathPlanner.shutdownAndExit(e);
	}
}
