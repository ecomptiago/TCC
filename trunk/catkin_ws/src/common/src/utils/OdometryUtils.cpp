/*
 * OdometryUtils.cpp
 *
 *  Created on: Sep 28, 2015
 *      Author: tcoelho
 */

#include "../../include/common/utils/OdometryUtils.h"

double OdometryUtils::getAngleFromQuaternation(tf::Quaternion quaternion,
	bool inRadian) {
	if(inRadian) {
		return tf::getYaw(quaternion);
	} else {
		return tf::getYaw(quaternion) * 180/M_PI;
	}
}
