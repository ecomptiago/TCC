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
		float xMax = occupancyGrid.info.origin.position.x
			+ (occupancyGrid.info.width * occupancyGrid.info.resolution);
		float yMax = occupancyGrid.info.origin.position.y
			+ (occupancyGrid.info.height * occupancyGrid.info.resolution);
		float xMin = occupancyGrid.info.origin.position.x;
		float yMin = occupancyGrid.info.origin.position.y;
		float cellSize = occupancyGrid.info.resolution;

		if(NumericUtils::isFirstLessEqual<float>(position.x, xMin) ||
			NumericUtils::isFirstGreaterEqual<float>(position.x, xMax) ||
			NumericUtils::isFirstLessEqual<float>(position.y, yMin) ||
			NumericUtils::isFirstGreaterEqual<float>(position.y,yMax)) {
				return -1;
		} else {
			int cellPosition = 0;
			for (float y = yMin; NumericUtils::isFirstLess<float>(y, yMax); y = y + cellSize) {
				for (float x = xMin; NumericUtils::isFirstLess<float>(x, xMax); x = x + cellSize) {
					bool posXGreaterCelX = NumericUtils::isFirstGreaterEqual<float>(position.x, x);
					bool posXLessCelX = NumericUtils::isFirstLessEqual<float>(position.x, x + cellSize);
					bool posYGreaterCelY = NumericUtils::isFirstGreaterEqual<float>(position.y, y);
					bool posYLessCelY = NumericUtils::isFirstLessEqual<float>(position.y, y + cellSize);
					if(posXGreaterCelX &&  posXLessCelX && posYGreaterCelY && posYLessCelY) {
						return cellPosition;
					}
					cellPosition++;
				}
			}
		}
		return -1;
}

void PathPlannerUtils::getCoordinatesFromDataVectorPosition(
	nav_msgs::OccupancyGrid& occupancyGrid, common::Position& position,
	int dataVectorPosition) {
		int totalRows = (occupancyGrid.info.height / occupancyGrid.info.resolution);
		int row = dataVectorPosition / totalRows;
		int column = dataVectorPosition - row * totalRows;
		double halfCell = occupancyGrid.info.resolution / 2;
		if(row == 0 ) {
			position.y = occupancyGrid.info.origin.position.y + halfCell;
		} else {
			position.y = occupancyGrid.info.origin.position.y + (row  * occupancyGrid.info.resolution) + halfCell;
		}
		if(column == 0) {
			position.x = occupancyGrid.info.origin.position.x + halfCell;
		} else {
			position.x = occupancyGrid.info.origin.position.x + (column * occupancyGrid.info.resolution) + halfCell;
		}
}
