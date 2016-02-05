/*
 * AStarGridCell.cpp
 *
 *  Created on: Feb 4, 2016
 *      Author: tcoelho
 */

#include "path_planner/search/AStar/AStarGridCell.h"

//Constructors
AStarGridCell::AStarGridCell(common::Position cellCoordinates,
	common::Position targetPosition, int cellGridPosition) :
	BaseGridCell(cellGridPosition){
		this->cellCoordinates = cellCoordinates;
		this->targetCoordinates = targetCoordinates;
		this->gCost = infiniteCost;
		this->comingFromCell = NULL;
}

AStarGridCell::AStarGridCell(): BaseGridCell() {
	this->gCost = infiniteCost;
	this->comingFromCell = NULL;
}

//Methods
bool AStarGridCell::calculateCellCost() {
	cost = gCost + sqrt(pow(targetCoordinates.x - cellCoordinates.x, 2) +
		pow(targetCoordinates.y - cellCoordinates.y, 2));
	return true;
}

void AStarGridCell::copy(AStarGridCell& aStarGridCell) {
	this->cellCoordinates = aStarGridCell.cellCoordinates;
	this->cellGridPosition = aStarGridCell.cellGridPosition;
	this->cost = aStarGridCell.cost;
	this->targetCoordinates = aStarGridCell.targetCoordinates;
}

void AStarGridCell::addNodeToNeighbours(int rightCell,nav_msgs::OccupancyGrid& occupancyGrid,
	std::vector<AStarGridCell>& neighbours) {
		common::Position position;
		PathPlannerUtils::getCoordinatesFromDataVectorPosition(occupancyGrid,
			position, rightCell);
		neighbours.push_back(
			AStarGridCell(position, targetCoordinates, rightCell));
}

void AStarGridCell::getCellNeighbours(std::vector<AStarGridCell> &neighbours,
	nav_msgs::OccupancyGrid &occupancyGrid) {
		int rightCell = cellGridPosition + 1;
		int leftCell = cellGridPosition - 1;
		int upperCell = cellGridPosition +
			(occupancyGrid.info.height / occupancyGrid.info.resolution);
		int belowCell = cellGridPosition -
			(occupancyGrid.info.height / occupancyGrid.info.resolution);
		if (rightCell < occupancyGrid.data.size()) {
			addNodeToNeighbours(rightCell, occupancyGrid, neighbours);
		}
		if (leftCell > 0) {
			addNodeToNeighbours(leftCell, occupancyGrid, neighbours);
		}
		if (upperCell < occupancyGrid.data.size()) {
			addNodeToNeighbours(upperCell, occupancyGrid, neighbours);
		}
		if (belowCell > 0) {
			addNodeToNeighbours(belowCell, occupancyGrid, neighbours);
		}
}

//Getters and setters
int AStarGridCell::getGCost() {
	return gCost;
}

void AStarGridCell::setGCost(int gCost) {
	this->gCost = gCost;
}

AStarGridCell*& AStarGridCell::getComingFromCell(){
	return comingFromCell;
}

void AStarGridCell::setComingFromCell(AStarGridCell &comingFromCell) {
	this->comingFromCell = &comingFromCell;
}

std::vector<AStarGridCell>& AStarGridCell::getSuccessors() {
	return successors;
}

void AStarGridCell::setSuccessors(std::vector<AStarGridCell>& successors) {
	this->successors = successors;
}
