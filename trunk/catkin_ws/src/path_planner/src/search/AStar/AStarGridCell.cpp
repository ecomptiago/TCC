/*
 * AStarGridCell.cpp
 *
 *  Created on: Feb 4, 2016
 *      Author: tcoelho
 */

#include "path_planner/search/AStar/AStarGridCell.h"

float AStarGridCell::getHeutisticValue() {
	return heutisticValue;
}

void AStarGridCell::setHeutisticValue(float heutisticValue) {
	this->heutisticValue = heutisticValue;
}

float AStarGridCell::calculateCellCost(int position) {
	return 0;
}
