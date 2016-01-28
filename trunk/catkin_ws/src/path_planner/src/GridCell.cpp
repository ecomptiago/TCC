/*
 * GridCell.cpp
 *
 *  Created on: Jan 28, 2016
 *      Author: tcoelho
 */

#include "path_planner/GridCell.h"

//Constructor
GridCell::GridCell() {
	this->position = -1;
	this->heuristicValue = -1;
	this->costValue = -1;
}

GridCell::GridCell(int position) {
	this->position = position;
	this->heuristicValue = -1;
	this->costValue = -1;
}

//Getters and setters
int GridCell::getCostValue() const {
	return costValue;
}

void GridCell::setCostValue(int costValue) {
	this->costValue = costValue;
}

float GridCell::getHeuristicValue() const {
	return heuristicValue;
}

void GridCell::setHeuristicValue(float heuristicValue) {
	this->heuristicValue = heuristicValue;
}

int GridCell::getPosition() const {
	return position;
}

void GridCell::setPosition(int position) {
	this->position = position;
}

//Methods
bool GridCell::calculateHeuristicValue(int targetPosition){
	return true;
}

