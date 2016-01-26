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

bool AStar::findPathToGoal(common::Position &position) {
	if(occupancyGridPointer == NULL) {
		return false;
	}
	return true;
}

void AStar::setOccupancyGrid(nav_msgs::OccupancyGrid &occupancyGrid) {
	occupancyGridPointer = &occupancyGrid;
}
