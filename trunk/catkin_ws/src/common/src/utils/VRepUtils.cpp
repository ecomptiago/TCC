/*
 * VRep.cpp
 *
 *  Created on: Dec 21, 2015
 *      Author: tiago
 */

#include "../../include/common/utils/VRepUtils.h"

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
