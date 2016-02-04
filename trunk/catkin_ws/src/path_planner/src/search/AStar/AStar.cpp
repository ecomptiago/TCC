/*
 * AStar.cpp
 *
 *  Created on: Jan 26, 2016
 *      Author: tcoelho
 */

#include "path_planner/search/AStar/AStar.h"

AStar::AStar() {
	this->occupancyGridPointer = NULL;
}

bool AStar::findPathToGoal(common::Position &initialCoordinates,
	common::Position &targetCoordinates) {
		if(occupancyGridPointer == NULL) {
			return false;
		} else {

			int initialCell =
				PathPlannerUtils::getDataVectorPosition(*occupancyGridPointer, initialCoordinates);
			int targetCell =
				PathPlannerUtils::getDataVectorPosition(*occupancyGridPointer, targetCoordinates);
			ROS_DEBUG("initial cell: %d target cell: %d",initialCell,targetCell);

			AStarGridCell aStarGridCell(initialCoordinates,targetCoordinates,initialCell);
			aStarGridCell.calculateCellCost(initialCell);
			openNodes[initialCell] = aStarGridCell;

			if(!openNodes.empty()) {
				AStarGridCell aStarGridCellSmallerCost;
				if(getCellWithSmallerCostOpenNodes(aStarGridCellSmallerCost)) {
					closedNodes[aStarGridCellSmallerCost.cellGridPosition] =
						aStarGridCellSmallerCost;
					openNodes.erase(aStarGridCellSmallerCost.cellGridPosition);
					return true;
				} else {
					return false;
				}

			} else {
				return false;
			}

		}
}

bool AStar::getCellWithSmallerCostOpenNodes(AStarGridCell &aStarGridCell) {
	AStarGridCell aStarGridCellSmalleCost;
	if(openNodes.empty()) {
		return false;
	}
	aStarGridCellSmalleCost.copy(openNodes[0]);
	for(int i = 1; i < openNodes.size(); i++) {
		if(NumericUtils::isFirstLess<float>(openNodes[i].cost,aStarGridCell.cost)) {
			aStarGridCellSmalleCost.copy(openNodes[0]);
		}
	}
	return true;
}

void AStar::setOccupancyGrid(nav_msgs::OccupancyGrid &occupancyGrid) {
	occupancyGridPointer = &occupancyGrid;
}
