/*
 * pathPlanner.cpp
 *
 *  Created on: Dec 15, 2015
 *      Author: tiago
 */

#include "path_planner/PathPlanner.h"

//Constructors
PathPlanner::PathPlanner(int argc, char **argv) :
	BaseRosNode(argc, argv, "Path_planner"){
}

//Methods
int PathPlanner::runNode() {
	ROS_INFO("Running node");

	if(VRepUtils::getObjectHandle(wallHandle,nodeHandler,signalObjectMap) &&
	   VRepUtils::getObjectHandle(cuboidHandle,nodeHandler,signalObjectMap)) {
			path_planner::simRosGetObjectGroupData simRosGetObjectGroupData;
			simRosGetObjectGroupData.request.objectType = sim_object_shape_type;
			//dataType = 2 retrieves the parent object handle
			simRosGetObjectGroupData.request.dataType = 2;
			serviceClientsMap[getObjectGroupDataService].call(simRosGetObjectGroupData);
			std::vector<int32_t> objectHandles =
				simRosGetObjectGroupData.response.handles;
			std::vector<int32_t> parentHandles =
				simRosGetObjectGroupData.response.intData;
			if (objectHandles.size() > 0 && parentHandles.size() > 0
				&& objectHandles.size() == parentHandles.size()) {
					for(int i = 0; i < parentHandles.size(); i++) {
					int parentHandle = parentHandles.at(i);
					if (parentHandle == signalObjectMap[cuboidHandle] ||
						parentHandle == signalObjectMap[wallHandle]) {
							if(!addToMap(objectHandles.at(i))) {
								ROS_ERROR("Failed to get object pose during the construction "
									"of the occupancy map.");
								shutdownAndExit();
							}
					}
				}
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
	return addServiceClient<path_planner::simRosGetObjectGroupData>(nodeHandler, getObjectGroupDataService);
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
	PathPlanner pathPlanner(argc,argv);
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
