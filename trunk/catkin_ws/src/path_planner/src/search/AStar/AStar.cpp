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
			aStarGridCell.setGCost(0);
			aStarGridCell.calculateCellCost();
			openNodes[initialCell] = aStarGridCell;

			while(!openNodes.empty()) {
				AStarGridCell aStarGridCellSmallerCost;
				getCellWithSmallerCostOpenNodes(aStarGridCellSmallerCost);
				closedNodes[aStarGridCellSmallerCost.cellGridPosition] =
					aStarGridCellSmallerCost;
				openNodes.erase(aStarGridCellSmallerCost.cellGridPosition);
				if(aStarGridCellSmallerCost.cellGridPosition ==  targetCell) {
					return true;
				} else {

					std::vector<AStarGridCell> neighbours(4);
					aStarGridCellSmallerCost.getCellNeighbours(neighbours,*occupancyGridPointer);

					std::vector<AStarGridCell>::iterator neighboursIterator;
					neighboursIterator = neighbours.begin();
					while(neighboursIterator != neighbours.end()) {
						((AStarGridCell)*neighboursIterator).setComingFromCell(aStarGridCellSmallerCost);
						if(occupancyGridPointer->data[((AStarGridCell)*neighboursIterator).cellGridPosition] == 100) {
							((AStarGridCell)*neighboursIterator).setGCost(infiniteCost);
						} else {
							((AStarGridCell)*neighboursIterator).setGCost(aStarGridCellSmallerCost.getGCost() + 1);
						}

						std::map<int,AStarGridCell>::iterator openNodesIterator =
							openNodes.find(((AStarGridCell)*neighboursIterator).cellGridPosition);
						std::map<int,AStarGridCell>::iterator closedNodesIterator =
							closedNodes.find(((AStarGridCell)*neighboursIterator).cellGridPosition);
					}
				}
			}
		}
		return true;
}

void AStar::getCellWithSmallerCostOpenNodes(AStarGridCell &aStarGridCell) {
	AStarGridCell aStarGridCellSmalleCost;
	aStarGridCellSmalleCost.copy(openNodes[0]);
	for(int i = 1; i < openNodes.size(); i++) {
		if(NumericUtils::isFirstLess<float>(openNodes[i].cost,aStarGridCell.cost)) {
			aStarGridCellSmalleCost.copy(openNodes[0]);
		}
	}
}

void AStar::setOccupancyGrid(nav_msgs::OccupancyGrid &occupancyGrid) {
	occupancyGridPointer = &occupancyGrid;
}
