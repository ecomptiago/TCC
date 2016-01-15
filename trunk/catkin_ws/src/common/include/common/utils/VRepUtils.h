/*
 * VRepUtils.h
 *
 *  Created on: Dec 21, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_COMMON_UTILS_VREPUTILS_H_
#define INCLUDE_COMMON_UTILS_VREPUTILS_H_

#include "map"
#include "iostream"
#include "ros/ros.h"
#include "../BaseRosNode.h"
#include "common/simRosGetObjectHandle.h"
#include "common/simRosGetObjectPose.h"


class VRepUtils {

	public:
		//Constructor
		VRepUtils() {};

		//Destructor
		virtual ~VRepUtils() {};

		//Methods
		static bool getObjectHandle(const char* objectHandleName,
			ros::NodeHandle &nodeHandle,
			std::map<std::string,int32_t> &signalObjectMap) {
				common::simRosGetObjectHandle simRosGetObjectHandle;
				simRosGetObjectHandle.request.objectName = objectHandleName;
				ros::ServiceClient client = nodeHandle.serviceClient
					<common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
				client.call(simRosGetObjectHandle);
				if(simRosGetObjectHandle.response.handle != -1) {
					ROS_DEBUG("Got %d int handle for %s", simRosGetObjectHandle.response.handle,
						objectHandleName);
					signalObjectMap[objectHandleName] = simRosGetObjectHandle.response.handle;
					return true;
				} else {
					return false;
				}
		}

		static bool getObjectPose(int32_t objectHandle,ros::NodeHandle &nodeHandle,
			common::simRosGetObjectPose &simRosGetObjectPose) {
				simRosGetObjectPose.request.handle = objectHandle;
				simRosGetObjectPose.request.relativeToObjectHandle = -1;
				ros::ServiceClient client = nodeHandle.serviceClient
					<common::simRosGetObjectPose>("/vrep/simRosGetObjectPose");
				client.call(simRosGetObjectPose);
				if(simRosGetObjectPose.response.result != -1) {
					return true;
				} else {
					return false;
				}
		}

		static void infoFailgetObjectsAndExit(const char* nodeName) {
			ROS_INFO("Failed to get objects from V-Rep simulation. Be sure"
				" that the simulation is running.");
			BaseRosNode::shutdownAndExit(nodeName);
		}
};

#endif /* INCLUDE_COMMON_UTILS_VREPUTILS_H_ */