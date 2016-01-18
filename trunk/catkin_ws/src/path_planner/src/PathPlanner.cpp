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

	if(VRepUtils::getObjectHandle(cuboidHandle,nodeHandler,signalObjectMap) &&
	   VRepUtils::getObjectHandle(floorHandle,nodeHandler,signalObjectMap)) {
			path_planner::simRosGetObjectGroupData simRosGetObjectGroupData;
			simRosGetObjectGroupData.request.objectType = sim_object_shape_type;
			//dataType = 2 retrieves the parent object handle
			simRosGetObjectGroupData.request.dataType = 2;
			serviceClientsMap[getObjectGroupDataService].call(simRosGetObjectGroupData);
			std::vector<int32_t> objectHandles =
				simRosGetObjectGroupData.response.handles;
			std::vector<int32_t> parentHandles =
				simRosGetObjectGroupData.response.intData;
			std::vector<int32_t> floorHandles(parentHandles.size());
			std::vector<int32_t> obstaclesHandles(parentHandles.size());
			if (objectHandles.size() > 0 && parentHandles.size() > 0
				&& objectHandles.size() == parentHandles.size()) {
					for(int i = 0; i < parentHandles.size(); i++) {
						int parentHandle = parentHandles.at(i);
						if(parentHandle == signalObjectMap[cuboidHandle]) {
							obstaclesHandles.push_back(parentHandle);
						} else if(parentHandle == signalObjectMap[floorHandle]) {
							floorHandles.push_back(parentHandle);
						}
					}
					buildOccupancyGrid(obstaclesHandles);
			} else {
				ROS_ERROR("There is not objects of shape type. "
					"Could not construct the map. Exiting!");
			}
	} else {
		ROS_ERROR("Failed to get objects from V-Rep simulation. Be sure"
			" that the simulation is running.");
		shutdownAndExit();
	}

	ros::spin();
	shutdownAndExit();
}

bool PathPlanner::buildOccupancyGrid(std::vector<int32_t> floorHandles){
	common::simRosGetObjectPose simRosGetObjectPose;
	path_planner::simRosGetObjectFloatParameter simRosGetObjectFloatParameter;
	std::vector<common::Position> positions(floorHandles.size());
	std::vector<path_planner::ObjectInfo> objectInfos(floorHandles.size());
	bool success = true;
	for(int i = 0; i < floorHandles.size(); i++) {
		int32_t floorHandle = floorHandles.at(i);
		if(VRepUtils::getObjectPose(floorHandle,nodeHandler,simRosGetObjectPose)) {
			path_planner::ObjectInfo objectInfo =
				getObjectInfo(floorHandle, simRosGetObjectPose);
			if(objectInfo.width != 0 && objectInfo.height != 0) {
				common::Position position;
				position.y = simRosGetObjectPose.response.pose.pose.position.y -
					objectInfo.height;
				position.x = simRosGetObjectPose.response.pose.pose.position.x -
					objectInfo.width;
				positions.push_back(position);
				objectInfos.push_back(objectInfo);
			} else {
				success = false;
			}
		} else {
			success = false;
		}
	}
	if(success) {

	}
	return success;
}

void PathPlanner::callGetFloatParameterService(int32_t objectHandle,
	int32_t parameterID,
	path_planner::simRosGetObjectFloatParameter simRosGetObjectFloatParameter) {
		simRosGetObjectFloatParameter.request.handle = objectHandle;
		simRosGetObjectFloatParameter.request.parameterID =
				parameterID;
		serviceClientsMap[getObjectFloatParameterService].call(
			simRosGetObjectFloatParameter);
}

path_planner::ObjectInfo PathPlanner::getObjectInfo(int32_t objectHandle,
	common::simRosGetObjectPose simRosGetObjectPose) {
		path_planner::ObjectInfo objectInfo;
		objectInfo.height = -1;
		objectInfo.width = -1;
		path_planner::simRosGetObjectFloatParameter simRosGetObjectModelBoxMinX;
		path_planner::simRosGetObjectFloatParameter simRosGetObjectModelBoxMaxX;
		callGetFloatParameterService(objectHandle, sim_objfloatparam_modelbbox_min_x,
			simRosGetObjectModelBoxMinX);
		callGetFloatParameterService(objectHandle, sim_objfloatparam_modelbbox_max_x,
			simRosGetObjectModelBoxMaxX);
		if(simRosGetObjectModelBoxMinX.response.result != -1 &&
			simRosGetObjectModelBoxMaxX.response.result != -1) {
				objectInfo.width = simRosGetObjectModelBoxMaxX.response.parameterValue -
					simRosGetObjectModelBoxMinX.response.parameterValue;
				path_planner::simRosGetObjectFloatParameter simRosGetObjectModelBoxMinY;
				path_planner::simRosGetObjectFloatParameter simRosGetObjectModelBoxMaxY;
				callGetFloatParameterService(objectHandle, sim_objfloatparam_modelbbox_min_y,
					simRosGetObjectModelBoxMinY);
				callGetFloatParameterService(objectHandle, sim_objfloatparam_modelbbox_max_y,
					simRosGetObjectModelBoxMaxY);
				if(simRosGetObjectModelBoxMinX.response.result != -1 &&
				   simRosGetObjectModelBoxMaxX.response.result != -1) {
						objectInfo.height = simRosGetObjectModelBoxMaxY.response.parameterValue -
							simRosGetObjectModelBoxMinY.response.parameterValue;
				}
		}
		return objectInfo;
}


bool PathPlanner::addToMap(int32_t objectHandle) {
	common::simRosGetObjectPose simRosGetObjectPose;
	if(VRepUtils::getObjectPose(objectHandle,nodeHandler,simRosGetObjectPose)) {
		tf::Quaternion quaternion(0, 0,
			simRosGetObjectPose.response.pose.pose.orientation.z,
			simRosGetObjectPose.response.pose.pose.orientation.w);
		double angle = OdometryUtils::getAngleFromQuaternation(quaternion,false);
		return false;
	} else{
		return false;
	}
}

bool PathPlanner::subscribeToTopics() {
	ROS_INFO("Subscribing to topics");
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

		   addServiceClient<path_planner::simRosGetObjectFloatParameter>(nodeHandler, getObjectFloatParameterService);
}

bool PathPlanner::createServiceServers() {
	return true;
}

bool PathPlanner::createPublishers() {
	ROS_INFO("Creating publishers");
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
