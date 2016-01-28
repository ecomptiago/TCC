/*
 * GridCell.cpp
 *
 *  Created on: Jan 28, 2016
 *      Author: tcoelho
 */

#include "path_planner/GridCell.h"

//Constructor
GridCell::GridCell() {
	this->x = -1;
	this->y = -1;
}

//Getters and setters
int GridCell::getX() const {
	return x;
}

void GridCell::setX(int x) {
	this->x = x;
}

int GridCell::getY() const {
	return y;
}

void GridCell::setY(int y) {
	this->y = y;
}



