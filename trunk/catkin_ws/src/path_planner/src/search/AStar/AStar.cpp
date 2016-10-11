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
				GridUtils::getDataVectorPosition(*occupancyGridPointer, initialCoordinates);
			int targetCell =
				GridUtils::getDataVectorPosition(*occupancyGridPointer, targetCoordinates);
			ROS_DEBUG("initial cell: %d target cell: %d",initialCell,targetCell);

			if(occupancyGridPointer->data[targetCell] == 0) { // 0 = freeCell
 				AStarGridCell aStarGridCell(targetCoordinates,initialCell);
				aStarGridCell.setGCost(0);
				aStarGridCell.calculateCellCoordinates(*occupancyGridPointer);
				aStarGridCell.calculateCellCost();
				openNodes.clear();
				closedNodes.clear();
				openNodes[initialCell] = aStarGridCell;

				while(!openNodes.empty()) {

					AStarGridCell aStarGridCellSmallerCost;
					getCellWithSmallerCostOpenNodes(aStarGridCellSmallerCost);

					std::vector<AStarGridCell> neighbours;
					aStarGridCellSmallerCost.getCellNeighbours(neighbours,*occupancyGridPointer);
					aStarGridCellSmallerCost.setSuccessors(neighbours);
					aStarGridCellSmallerCost.setTargetCoordinates(targetCoordinates);
					aStarGridCellSmallerCost.calculateCellCoordinates(*occupancyGridPointer);

					ROS_DEBUG("Got node with position %d and cost %f",aStarGridCellSmallerCost.cellGridPosition, aStarGridCellSmallerCost.cost);
					closedNodes[aStarGridCellSmallerCost.cellGridPosition] =
						aStarGridCellSmallerCost;
					openNodes.erase(aStarGridCellSmallerCost.cellGridPosition);
					if(aStarGridCellSmallerCost.cellGridPosition ==  targetCell) {
						return true;
					} else {
						for(int i = 0; i < aStarGridCellSmallerCost.getSuccessors().size(); i++) {
							AStarGridCell aStarGridCellNeighbour = aStarGridCellSmallerCost.getSuccessors()[i];
							std::map<int,AStarGridCell>::iterator closedNodesIterator =
								closedNodes.find(aStarGridCellNeighbour.cellGridPosition);
							if(closedNodesIterator != closedNodes.end()) {
								continue;
							} else {
								if(occupancyGridPointer->data[aStarGridCellNeighbour.cellGridPosition] == 100) {
									aStarGridCellNeighbour.setGCost(infiniteCost);
								} else {
									aStarGridCellNeighbour.setGCost(aStarGridCellSmallerCost.getGCost() + 1);
								}
								aStarGridCellNeighbour.setTargetCoordinates(targetCoordinates);
								aStarGridCellNeighbour.calculateCellCoordinates(*occupancyGridPointer);
								aStarGridCellNeighbour.calculateCellCost();
								openNodes[aStarGridCellNeighbour.cellGridPosition] = aStarGridCellNeighbour;
							}
						}
					}
				}
			} else {
				ROS_DEBUG("Target cell is occupied or unknown");
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
			tempAStarGridCell.copy(aStarGridCellMapPair.second);
		}
		openNodesIterator++;
	}
	aStarGridCell.copy(tempAStarGridCell);
}

std::vector<int> AStar::optimizePath(std::vector<AStarGridCell>& path) {
	int i = 0;
	int gridLengh = occupancyGridPointer->info.width
			/ occupancyGridPointer->info.resolution;
	int freeCell = 0;
	std::vector<int> cellsToEliminate(path.size());
	while (i < path.size()) {
		AStarGridCell actualCell = path.at(i);
		if(i + 1 >= path.size() || i + 2 >= path.size()) {
			break;
		}
		AStarGridCell nextCell = path.at(i + 1);
		AStarGridCell nextNextCell = path.at(i + 2);
		int upperCellFromActualCell = actualCell.cellGridPosition + gridLengh;
		int upperRightCellFromActualCell = upperCellFromActualCell + 1;
		int upperLeftCellFromActualCell = upperCellFromActualCell - 1;
		int aboveCellFromActualCell = actualCell.cellGridPosition - gridLengh;
		int aboveRightCellFromActualCell = aboveCellFromActualCell + 1;
		int aboveLeftCellFromActualCell = aboveCellFromActualCell - 1;
		int righCellFromActualCell = actualCell.cellGridPosition + 1;
		int leftCellFromActualCell = actualCell.cellGridPosition - 1;

		if(nextNextCell.cellGridPosition == upperRightCellFromActualCell) {
			if ((nextCell.cellGridPosition == upperCellFromActualCell &&
				 occupancyGridPointer->data[righCellFromActualCell] == freeCell) ||
				(nextCell.cellGridPosition == righCellFromActualCell &&
				 occupancyGridPointer->data[upperCellFromActualCell] == freeCell)) {
						cellsToEliminate.push_back(nextCell.cellGridPosition);
						i = i + 2;
						continue;
			}
		}
		if(nextNextCell.cellGridPosition == upperLeftCellFromActualCell) {
			if ((nextCell.cellGridPosition == upperCellFromActualCell &&
				 occupancyGridPointer->data[leftCellFromActualCell] == freeCell) ||
				(nextCell.cellGridPosition == leftCellFromActualCell &&
				 occupancyGridPointer->data[upperCellFromActualCell] == freeCell)) {
						cellsToEliminate.push_back(nextCell.cellGridPosition);
						i = i + 2;
						continue;
			}
		}
		if(nextNextCell.cellGridPosition == aboveRightCellFromActualCell) {
			if ((nextCell.cellGridPosition == aboveCellFromActualCell &&
				 occupancyGridPointer->data[righCellFromActualCell] == freeCell) ||
				(nextCell.cellGridPosition == righCellFromActualCell &&
				 occupancyGridPointer->data[aboveCellFromActualCell] == freeCell)) {
						cellsToEliminate.push_back(nextCell.cellGridPosition);
						i = i + 2;
						continue;
			}
		}
		if(nextNextCell.cellGridPosition == aboveLeftCellFromActualCell) {
			if ((nextCell.cellGridPosition == aboveCellFromActualCell &&
				 occupancyGridPointer->data[leftCellFromActualCell] == freeCell) ||
				(nextCell.cellGridPosition == leftCellFromActualCell &&
				 occupancyGridPointer->data[aboveCellFromActualCell] == freeCell)) {
						cellsToEliminate.push_back(nextCell.cellGridPosition);
						i = i + 2;
						continue;
			}
		}

		i++;
	}
	return cellsToEliminate;
}

//TODO- This should be optimized
void AStar::reconstructPath(std::vector<AStarGridCell> &path, common::Position &targetCoordinates, common::Position &initialCoordinates) {
	int initialCell = GridUtils::getDataVectorPosition(*occupancyGridPointer, initialCoordinates);
	int targetCell = GridUtils::getDataVectorPosition(*occupancyGridPointer, targetCoordinates);

	std::map<int,AStarGridCell>::iterator closedNodesIterator =
		closedNodes.find(targetCell);
	if(closedNodesIterator != closedNodes.end()) {

		path.push_back(closedNodes[targetCell]);
		while(path.back().cellGridPosition != initialCell) {
			getCellWithSmallerCostAndSuccessor(path.back().cellGridPosition, path);
		}

		std::reverse(path.begin(),path.end());

		std::vector<AStarGridCell>::iterator it;
		it = path.begin();
		int charsWrote = 0;
		char buffer [path.size() * 6];

		while(it != path.end()) {
			charsWrote += sprintf(buffer + charsWrote,
					" %d,",((AStarGridCell)*it).cellGridPosition);
			it++;
		}
		ROS_DEBUG("Path found: [%s]",buffer);

		std::vector<int> cellsToEliminate = optimizePath(path);

		std::vector<int>::iterator itCellElimination;
		itCellElimination = cellsToEliminate.begin();

		while(itCellElimination != cellsToEliminate.end()) {
			for(int i = 0; i < path.size(); i++) {
				if(path.at(i).cellGridPosition == *itCellElimination) {
					path.erase(path.begin() + i);
					break;
				}
			}
			itCellElimination++;
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
		ROS_DEBUG("Smaller cost is %f from cell %d",tempAStarGridCell.cost, tempAStarGridCell.cellGridPosition);
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
