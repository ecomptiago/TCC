/*
 * PathPlannerUtils.cpp
 *
 *  Created on: Jan 28, 2016
 *      Author: tcoelho
 */

#include "path_planner/utils/PathPlannerUtils.h"

//GridCell PathPlannerUtils::getCellFromPosition(common::Position& position) {
//	GridCell gridCell();
//	return gridCell;
//}


int PathPlannerUtils::getDataVectorPosition(nav_msgs::OccupancyGrid &occupancyGrid,
	common::Position &position) {
		float cellResolution = occupancyGrid.info.resolution;
		float yCell = NumericUtils::round(position.y - occupancyGrid.info.origin.position.y,0.6);
		float xCell = NumericUtils::round(position.x - occupancyGrid.info.origin.position.x,0.6);
		float wCell = NumericUtils::round(occupancyGrid.info.width,0.6);
		return ((wCell / cellResolution) * (yCell / cellResolution)) + (xCell / cellResolution);
}
