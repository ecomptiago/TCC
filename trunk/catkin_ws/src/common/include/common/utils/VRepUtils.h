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

#define poseRelativeToWorld -1
#define responseError -1

class VRepUtils {

	public:
		//Constructor
		VRepUtils() {};

		//Destructor
		virtual ~VRepUtils() {};

		//Methods
		static bool getObjectHandle(const char* objectHandleName,
			ros::NodeHandle &nodeHandle,
			std::map<std::string,int32_t> &signalObjectMap);

		static bool getObjectPose(int32_t objectHandle,ros::NodeHandle &nodeHandle,
			common::simRosGetObjectPose &simRosGetObjectPose);

};

#endif /* INCLUDE_COMMON_UTILS_VREPUTILS_H_ */
