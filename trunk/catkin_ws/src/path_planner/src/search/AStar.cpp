/*
 * AStar.cpp
 *
 *  Created on: Jan 26, 2016
 *      Author: tcoelho
 */

#include "path_planner/search/AStar.h"

AStar::AStar() {
	this->occupancyGridPointer = NULL;
}

bool AStar::findPathToGoal(common::Position &initialPosition,
	common::Position &targetPosition) {
		if(occupancyGridPointer == NULL) {
			return false;
		} else {
			ROS_DEBUG("initial: %d final: %d",
				PathPlannerUtils::getDataVectorPosition(*occupancyGridPointer, initialPosition),
				PathPlannerUtils::getDataVectorPosition(*occupancyGridPointer, targetPosition));
//			GridCell gridCell(1);
//			gridCell.calculateHeuristicValue(0);

			return true;
		}
}

void AStar::setOccupancyGrid(nav_msgs::OccupancyGrid &occupancyGrid) {
	occupancyGridPointer = &occupancyGrid;
}
