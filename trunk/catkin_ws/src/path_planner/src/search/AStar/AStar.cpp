/*
 * AStar.cpp
 *
 *  Created on: Jan 26, 2016
 *      Author: tcoelho
 */

#include "path_planner/search/AStar/AStar.h"

//Constructor
AStar::AStar() {
	this->occupancyGridPointer = NULL;
}

//Methods
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
				std::map<int,AStarGridCell>::iterator openNodesIterator = openNodes.begin();
				while(openNodesIterator != openNodes.end()) {
					std::pair<const int, AStarGridCell> aStarGridCellMapPair(*openNodesIterator);
					ROS_DEBUG("open node : grid position %d cost %f",aStarGridCellMapPair.second.cellGridPosition, aStarGridCellMapPair.second.cost);
					for(int i = 0; i < aStarGridCellMapPair.second.getSuccessors().size(); i++) {
						ROS_DEBUG("open node %d sucessor position $%d cost %f", aStarGridCellMapPair.second.cellGridPosition,
							aStarGridCellMapPair.second.getSuccessors()[i].cellGridPosition,
							aStarGridCellMapPair.second.getSuccessors()[i].cost);
					}
					openNodesIterator++;
				}

				std::map<int,AStarGridCell>::iterator closedNodesIterator = closedNodes.begin();
				while(closedNodesIterator != closedNodes.end()) {
					std::pair<const int, AStarGridCell> aStarGridCellMapPair(*closedNodesIterator);
					ROS_DEBUG("closed node : grid position %d cost %f",aStarGridCellMapPair.second.cellGridPosition, aStarGridCellMapPair.second.cost);
					for(int i = 0; i < aStarGridCellMapPair.second.getSuccessors().size(); i++) {
						ROS_DEBUG("closed node %d sucessor position $%d cost %f", aStarGridCellMapPair.second.cellGridPosition,
							aStarGridCellMapPair.second.getSuccessors()[i].cellGridPosition,
							aStarGridCellMapPair.second.getSuccessors()[i].cost);
					}
					closedNodesIterator++;
				}

				AStarGridCell aStarGridCellSmallerCost;
				getCellWithSmallerCostOpenNodes(aStarGridCellSmallerCost);

				std::vector<AStarGridCell> neighbours;
				aStarGridCellSmallerCost.getCellNeighbours(neighbours,*occupancyGridPointer);
				aStarGridCellSmallerCost.setSuccessors(neighbours);

				ROS_DEBUG("Got node with position %d and cost %f",aStarGridCellSmallerCost.cellGridPosition, aStarGridCellSmallerCost.cost);
				closedNodes[aStarGridCellSmallerCost.cellGridPosition] =
					aStarGridCellSmallerCost;
				openNodes.erase(aStarGridCellSmallerCost.cellGridPosition);
				if(aStarGridCellSmallerCost.cellGridPosition ==  targetCell) {
					return true;
				} else {
					for(int i = 0; i < aStarGridCellSmallerCost.getSuccessors().size(); i++) {
						AStarGridCell aStarGridCellNeighbour = aStarGridCellSmallerCost.getSuccessors()[i];
						std::map<int,AStarGridCell>::iterator openNodesIterator =
							openNodes.find(aStarGridCellNeighbour.cellGridPosition);
						std::map<int,AStarGridCell>::iterator closedNodesIterator =
							closedNodes.find(aStarGridCellNeighbour.cellGridPosition);
						if(closedNodesIterator != closedNodes.end()) {
							continue;
						} else if(openNodesIterator == openNodes.end()) {
							if(occupancyGridPointer->data[aStarGridCellNeighbour.cellGridPosition] == 100) {
								aStarGridCellNeighbour.setGCost(infiniteCost);
							} else {
								aStarGridCellNeighbour.setGCost(aStarGridCellSmallerCost.getGCost() + 1);
							}
							aStarGridCellNeighbour.calculateCellCost();
							openNodes[aStarGridCellNeighbour.cellGridPosition] = aStarGridCellNeighbour;
						}
					}
				}
			}
		}
		return false;
}

void AStar::getCellWithSmallerCostOpenNodes(AStarGridCell &aStarGridCell) {
	std::map<int,AStarGridCell>::iterator openNodesIterator = openNodes.begin();
	AStarGridCell tempAStarGridCell;
	bool firstCell = true;
	while(openNodesIterator != openNodes.end()) {
		std::pair<const int, AStarGridCell> aStarGridCellMapPair(*openNodesIterator);
		if(firstCell) {
			tempAStarGridCell.copy(aStarGridCellMapPair.second);
			firstCell = false;
		} else if(NumericUtils::isFirstLess(aStarGridCellMapPair.second.cost,tempAStarGridCell.cost)) {
			ROS_DEBUG("Changing node %d with cost %f with node %d with cost %f", aStarGridCellMapPair.second.cellGridPosition,
				aStarGridCellMapPair.second.cost,tempAStarGridCell.cellGridPosition,tempAStarGridCell.cost);
			tempAStarGridCell.copy(aStarGridCellMapPair.second);
		}
		openNodesIterator++;
	}
	aStarGridCell.copy(tempAStarGridCell);
}

//TODO- This should be optimized
void AStar::reconstructPath(std::vector<AStarGridCell> &path, common::Position &targetCoordinates, common::Position &initialCoordinates) {
	int initialCell = PathPlannerUtils::getDataVectorPosition(*occupancyGridPointer, initialCoordinates);
	int targetCell = PathPlannerUtils::getDataVectorPosition(*occupancyGridPointer, targetCoordinates);

	std::map<int,AStarGridCell>::iterator closedNodesIterator =
		closedNodes.find(targetCell);
	if(closedNodesIterator != closedNodes.end()) {

		path.push_back(closedNodes[targetCell]);
		while(path.back().cellGridPosition != initialCell) {
			getCellWithSmallerCostAndSuccessor(path.back().cellGridPosition, path);
		}
	} else {
		ROS_ERROR("Target node %d not in closed nodes", targetCell);
	}
}

void AStar::getCellWithSmallerCostAndSuccessor(int targetCell, std::vector<AStarGridCell> &path) {
	std::map<int,AStarGridCell>::iterator closedNodesIterator = closedNodes.begin();
	AStarGridCell tempAStarGridCell;
	tempAStarGridCell.calculateCellCost();
	while(closedNodesIterator != closedNodes.end()) {
	std::pair<const int, AStarGridCell> aStarGridCellMapPair(*closedNodesIterator);
		if(successorsContainsCell(aStarGridCellMapPair, targetCell)) {
			if(NumericUtils::isFirstLessEqual(aStarGridCellMapPair.second.cost, tempAStarGridCell.cost)) {
				tempAStarGridCell.copy(aStarGridCellMapPair.second);
			}
		}
		closedNodesIterator++;
	}
	path.push_back(tempAStarGridCell);
}

bool AStar::successorsContainsCell(std::pair<const int, AStarGridCell> &aStarGridCell, int targetCell) {
	for(int i = 0; i < aStarGridCell.second.getSuccessors().size(); i++) {
		if(aStarGridCell.second.getSuccessors()[i].cellGridPosition == targetCell) {
			return true;
		}
	}
	return false;
}

void AStar::setOccupancyGrid(nav_msgs::OccupancyGrid &occupancyGrid) {
	occupancyGridPointer = &occupancyGrid;
}
