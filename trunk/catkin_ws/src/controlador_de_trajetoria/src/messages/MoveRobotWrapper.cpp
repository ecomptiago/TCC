/*
 * MoveRobotWrapper.cpp
 *
 *  Created on: Sep 16, 2015
 *      Author: tcoelho
 */

#include "controlador_de_trajetoria/messages/MoveRobotWrapper.h"

MoveRobotWrapper::MoveRobotWrapper() {
	id = -1;
}

MoveRobotWrapper::MoveRobotWrapper(
	const controlador_de_trajetoria::Move_robot::ConstPtr& moveRobotPosition) {
		id = idCounter;
		id++;
		moveRobotObj = *moveRobotPosition;
}

MoveRobotWrapper::MoveRobotWrapper(
	controlador_de_trajetoria::Move_robot moveRobotPosition) {
		id = idCounter;
		id++;
		moveRobotObj = moveRobotPosition;
}

int MoveRobotWrapper::getId() const {
	return id;
}

const controlador_de_trajetoria::Move_robot& MoveRobotWrapper::getMoveRobotObj() const {
	return moveRobotObj;
}
