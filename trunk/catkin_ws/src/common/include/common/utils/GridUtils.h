/*
 * GridUtils.h
 *
 *  Created on: Oct 11, 2016
 *      Author: tcoelho
 */

#ifndef INCLUDE_COMMON_UTILS_GRIDUTILS_H_
#define INCLUDE_COMMON_UTILS_GRIDUTILS_H_

#include "math.h"
#include "nav_msgs/OccupancyGrid.h"
#include "common/Position.h"
#include "common/utils/NumericUtils.h"

class GridUtils {

	public:

		//Destructor
		virtual ~GridUtils() {};

		//Methods
		static int getDataVectorPosition(nav_msgs::OccupancyGrid &occupancyGrid,
			common::Position &position);
		static void getCoordinatesFromDataVectorPosition(nav_msgs::OccupancyGrid &occupancyGrid,
			common::Position &position, int dataVectorPosition);

};


#endif /* INCLUDE_COMMON_UTILS_GRIDUTILS_H_ */
