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

		double cellResolution = occupancyGrid.info.resolution;

		//TODO - This should be improved
		int yCell = 0;
		double yStartInterval = occupancyGrid.info.origin.position.y;
		double yStopInterval = yStartInterval + cellResolution;
		double yObjectPosition = position.y;
		double yLastStopInterval = occupancyGrid.info.origin.position.y +
			occupancyGrid.info.height / cellResolution;
		while( !NumericUtils::isFirstGreaterEqualWithPrecision(yStopInterval,yLastStopInterval,2) &&
			!(NumericUtils::isFirstGreaterEqualWithPrecision(yObjectPosition,yStartInterval,2) &&
			  NumericUtils::isFirstLessWithPrecision(yObjectPosition,yStopInterval,2))) {
				yStartInterval = yStopInterval;
				yStopInterval = yStartInterval + cellResolution;
				yCell++;
		}

		//TODO - This should be improved
		int xCell = 0;
		double xStartInterval = occupancyGrid.info.origin.position.x;
		double xStopInterval = xStartInterval + cellResolution;
		double xObjectPosition = position.x;
		double xLastStopInterval = occupancyGrid.info.origin.position.x +
			occupancyGrid.info.width / cellResolution;
		while(!NumericUtils::isFirstGreaterEqualWithPrecision(xStopInterval,xLastStopInterval,2) &&
			!(NumericUtils::isFirstGreaterEqualWithPrecision(xObjectPosition,xStartInterval,2) &&
			  NumericUtils::isFirstLessWithPrecision(xObjectPosition,xStopInterval,2))) {
				xStartInterval = xStopInterval;
				xStopInterval = xStartInterval + cellResolution;
				xCell++;
		}

		return ((occupancyGrid.info.width / cellResolution) * yCell) + xCell;
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
