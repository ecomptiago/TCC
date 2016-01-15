/*
 * pathPlanner.cpp
 *
 *  Created on: Dec 15, 2015
 *      Author: tiago
 */

#include "path_planner/PathPlanner.h"

//Constructors
PathPlanner::PathPlanner(int argc, char **argv) :
	BaseRosNode(argc, argv, nodeName){
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
			std::vector<int, std::allocator<int>> responseHandles =
				simRosGetObjectGroupData.response.handles;
			std::vector<int, std::allocator<int>> responseIntData =
				simRosGetObjectGroupData.response.intData;
			if (responseHandles.size() > 0 && responseIntData.size() > 0
				&& responseHandles.size() == responseIntData.size()) {
				std::vector<int32_t> wallsVectorObjectHandles(responseIntData.size());
				std::vector<int32_t> cuboidVectorHandles(responseIntData.size());
				for(int i = 0; i < responseIntData.size(); i++) {
					if(responseIntData.at(i) == signalObjectMap[cuboidHandle]) {
						cuboidVectorHandles.push_back(responseIntData.at(i));
					} else if(responseIntData.at(i) == signalObjectMap[wallHandle]) {
						wallsVectorObjectHandles.push_back(responseIntData.at(i));
					}
				}
			} else {
				ROS_INFO("There is not objects of shape type. "
					"Could not construct the map. Exiting!");
			}
	} else {
		VRepUtils::infoFailgetObjectsAndExit(nodeName);
	}

	ros::spin();
	BaseRosNode::shutdownAndExit(nodeName);
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

int PathPlanner::infoFailAndExit(const char* topicName) {
	ROS_INFO("Failed to create %s topic",topicName);
	return BaseRosNode::shutdownAndExit(nodeName);
}

bool PathPlanner::createPublishers() {
	ROS_INFO("Creating publishers");
	return true;
}

//Callback

//Main
int main(int argc, char **argv) {
	PathPlanner rosAriaVRep(argc,argv);
	try{
		if(rosAriaVRep.createServices() &&
			rosAriaVRep.subscribeToTopics() &&
			rosAriaVRep.createPublishers()) {
				return rosAriaVRep.runNode();
		} else {
			BaseRosNode::shutdownAndExit(nodeName);
		}
	} catch (std::exception &e) {
		BaseRosNode::shutdownAndExit(nodeName);
	}
}
