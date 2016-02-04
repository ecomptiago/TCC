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
}

AStarGridCell::AStarGridCell(): BaseGridCell() {
}

//Methods
bool AStarGridCell::calculateCellCost(int initialPosition) {

	float costToGetToCell;
	float heuristicCost;

	if(initialPosition == cellGridPosition) {
		costToGetToCell = 0;
	} else {

	}

	heuristicCost = sqrt(pow(targetCoordinates.x - cellCoordinates.x, 2) +
		pow(targetCoordinates.y - cellCoordinates.y, 2));
	cost = costToGetToCell + heuristicCost;


	return true;
}

void AStarGridCell::copy(AStarGridCell& aStarGridCell) {
	this->cellCoordinates = aStarGridCell.cellCoordinates;
	this->cellGridPosition = aStarGridCell.cellGridPosition;
	this->cost = aStarGridCell.cost;
	this->targetCoordinates = aStarGridCell.targetCoordinates;
}

