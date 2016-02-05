/*
 * PathPlannerUtils.cpp
 *
 *  Created on: Jan 28, 2016
 *      Author: tcoelho
 */

#include "path_planner/utils/PathPlannerUtils.h"

//Methods
int PathPlannerUtils::getDataVectorPosition(nav_msgs::OccupancyGrid &occupancyGrid,
	common::Position &position) {
		float cellResolution = occupancyGrid.info.resolution;
		float yCell = NumericUtils::round(position.y - occupancyGrid.info.origin.position.y,0.6);
		float xCell = NumericUtils::round(position.x - occupancyGrid.info.origin.position.x,0.6);
		float wCell = NumericUtils::round(occupancyGrid.info.width,0.6);
		return ((wCell / cellResolution) * (yCell / cellResolution)) + (xCell / cellResolution);
}

void PathPlannerUtils::getCoordinatesFromDataVectorPosition(
	nav_msgs::OccupancyGrid& occupancyGrid, common::Position& position,
	int dataVectorPosition) {
		int row = dataVectorPosition / (occupancyGrid.info.height / occupancyGrid.info.resolution);
		if(row == 0) {
			row++;
		}
		int column = dataVectorPosition / (occupancyGrid.info.width / occupancyGrid.info.resolution);
		if(column == 0) {
			column++;
		}
		position.y = occupancyGrid.info.origin.position.y + (row * (occupancyGrid.info.resolution / 2));
		position.x = occupancyGrid.info.origin.position.x + (column * (occupancyGrid.info.resolution / 2));
}
