/*
 * OdometryUtils.h
 *
 *  Created on: Sep 28, 2015
 *      Author: tcoelho
 */

#ifndef INCLUDE_COMMON_UTILS_ODOMETRYUTILS_H_
#define INCLUDE_COMMON_UTILS_ODOMETRYUTILS_H_

#include "tf/transform_datatypes.h"
#include "geometry_msgs/Quaternion.h"

class OdometryUtils {

	public:
		//Constructor
		OdometryUtils() {};

		//Destructor
		virtual ~OdometryUtils() {};

		//Methods
		template<class T>
		static double getAngleFromQuaternation(T quaternion) {
			return tf::getYaw(quaternion) * 180/M_PI;
		}

};

#endif /* INCLUDE_COMMON_UTILS_ODOMETRYUTILS_H_ */
