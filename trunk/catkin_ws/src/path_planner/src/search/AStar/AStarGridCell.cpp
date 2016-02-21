/*
 * AStarGridCell.cpp
 *
 *  Created on: Feb 4, 2016
 *      Author: tcoelho
 */

#include "path_planner/search/AStar/AStarGridCell.h"

//Constructors
AStarGridCell::AStarGridCell(common::Position &targetCoordinates,
	int cellGridPosition) : BaseGridCell(cellGridPosition){
		this->targetCoordinates = targetCoordinates;
		this->gCost = infiniteCost;
}

AStarGridCell::AStarGridCell(): BaseGridCell() {
	this->gCost = infiniteCost;
}

AStarGridCell::AStarGridCell(int cellGridPosition):
	BaseGridCell(cellGridPosition) {
		this->gCost = infiniteCost;
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
	this->gCost = aStarGridCell.gCost;
	if(aStarGridCell.getSuccessors().size() > 0){
		setSuccessors(aStarGridCell.getSuccessors());
	}
}


void AStarGridCell::calculateCellCoordinates(nav_msgs::OccupancyGrid& occupancyGrid) {
	if(cellGridPosition != -1) {
		common::Position cellCoordinates;
		PathPlannerUtils::getCoordinatesFromDataVectorPosition(occupancyGrid,cellCoordinates,cellGridPosition);
		this->cellCoordinates = cellCoordinates;
	}
}

void AStarGridCell::addNodeToNeighbours(int cellGridPosition,nav_msgs::OccupancyGrid& occupancyGrid,
	std::vector<AStarGridCell>& neighbours) {
		common::Position position;
		PathPlannerUtils::getCoordinatesFromDataVectorPosition(occupancyGrid,
			position, cellGridPosition);
		neighbours.push_back(AStarGridCell(targetCoordinates, cellGridPosition));
}

bool AStarGridCell::hasRightCell(int cellGridPosition, int columns, int rows) {
	int lastCellFirstRow = (columns - 1);
	for(int i = 0; i < rows; i++) {
		if(cellGridPosition == lastCellFirstRow + (i * columns)) {
			return false;
		}
	}
	return true;
}

void AStarGridCell::getCellNeighbours(std::vector<AStarGridCell> &neighbours,
	nav_msgs::OccupancyGrid &occupancyGrid) {
		int columns = occupancyGrid.info.width / occupancyGrid.info.resolution;
		int rows = occupancyGrid.info.height / occupancyGrid.info.resolution;
		//Verifying if cell has an upper cell
		if(cellGridPosition < (columns * (rows - 1))) {
			addNodeToNeighbours(cellGridPosition + columns, occupancyGrid, neighbours);
		}
		//Verifying if cell has a cell below it
		if(cellGridPosition > columns - 1) {
			addNodeToNeighbours(cellGridPosition - columns, occupancyGrid, neighbours);
		}
		//Verifying if cell has a cell in the right
		//TODO - improve this
		if(hasRightCell(cellGridPosition,columns,rows)) {
			addNodeToNeighbours(cellGridPosition + 1, occupancyGrid, neighbours);
		}
		//Verifying if cell has a cell in the left
		if(cellGridPosition % columns != 0) {
			addNodeToNeighbours(cellGridPosition - 1, occupancyGrid, neighbours);
		}
}

//Getters and setters
int AStarGridCell::getGCost() {
	return gCost;
}

void AStarGridCell::setGCost(int gCost) {
	this->gCost = gCost;
}

std::vector<AStarGridCell>& AStarGridCell::getSuccessors() {
	return successors;
}

void AStarGridCell::setSuccessors(std::vector<AStarGridCell>& successors) {
	this->successors.resize(successors.size());
	for(int i = 0; i < successors.size(); i++) {
		this->successors.at(i) = successors.at(i);
	}
}

common::Position& AStarGridCell::getTargetCoordinates() {
	return targetCoordinates;
}

common::Position& AStarGridCell::getCellCoordinates() {
	return cellCoordinates;
}

void AStarGridCell::setTargetCoordinates(common::Position& targetCoordinates) {
	this->targetCoordinates = targetCoordinates;
}
