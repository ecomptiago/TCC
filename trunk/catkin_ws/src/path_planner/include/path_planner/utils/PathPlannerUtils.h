/*
 * PathPlannerUtils.h
 *
 *  Created on: Jan 28, 2016
 *      Author: tcoelho
 */

#ifndef PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_UTILS_PATHPLANNERUTILS_H_
#define PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_UTILS_PATHPLANNERUTILS_H_

#include "nav_msgs/OccupancyGrid.h"
#include "common/Position.h"
#include "common/utils/NumericUtils.h"

class PathPlannerUtils {

	public:

		//Destructor
		virtual ~PathPlannerUtils() {};

		//Methods
//		static GridCell getCellFromPosition(common::Position &position);

		static int getDataVectorPosition(nav_msgs::OccupancyGrid &occupancyGrid,
			common::Position &position);
};

#endif /* PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_UTILS_PATHPLANNERUTILS_H_ */
