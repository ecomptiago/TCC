/*
 * VRep.cpp
 *
 *  Created on: Dec 21, 2015
 *      Author: tiago
 */

#include "../../include/common/utils/VRepUtils.h"

bool VRepUtils::getObjectHandle(const char* objectHandleName,
	ros::NodeHandle &nodeHandle,std::map<std::string,int32_t> &signalObjectMap) {
		common::simRosGetObjectHandle simRosGetObjectHandle;
		simRosGetObjectHandle.request.objectName = objectHandleName;
		ros::ServiceClient client = nodeHandle.serviceClient
			<common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
		client.call(simRosGetObjectHandle);
		if(simRosGetObjectHandle.response.handle != responseError) {
			if(simRosGetObjectHandle.response.handle == 0) {
				ROS_WARN("Got %d int handle for %s", simRosGetObjectHandle.response.handle,
					objectHandleName);
			}
			ROS_DEBUG("Got %d int handle for %s", simRosGetObjectHandle.response.handle,
				objectHandleName);
			signalObjectMap[objectHandleName] = simRosGetObjectHandle.response.handle;
			return true;
		} else {
			return false;
		}
}

bool VRepUtils::getObjectPose(int32_t objectHandle,ros::NodeHandle &nodeHandle,
	common::simRosGetObjectPose &simRosGetObjectPose) {
		simRosGetObjectPose.request.handle = objectHandle;
		simRosGetObjectPose.request.relativeToObjectHandle = poseRelativeToWorld;
		ros::ServiceClient client = nodeHandle.serviceClient
			<common::simRosGetObjectPose>("/vrep/simRosGetObjectPose");
		client.call(simRosGetObjectPose);
		if(simRosGetObjectPose.response.result != responseError) {
			return true;
		} else {
			return false;
		}
}
