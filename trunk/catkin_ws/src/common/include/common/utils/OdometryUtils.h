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
		static double getAngleFromQuaternation(tf::Quaternion quaternion,
			bool inRadian);

};

#endif /* INCLUDE_COMMON_UTILS_ODOMETRYUTILS_H_ */
