/*
 * OdometryUtils.cpp
 *
 *  Created on: Sep 28, 2015
 *      Author: tcoelho
 */

#include "../../include/common/utils/OdometryUtils.h"

double OdometryUtils::getAngleFromQuaternation(geometry_msgs::Quaternion quaternion) {
	return tf::getYaw(quaternion) * 180/M_PI;
}

double OdometryUtils::getAngleFromQuaternation(tf::Quaternion quaternion) {
	return tf::getYaw(quaternion) * 180/M_PI;
}
