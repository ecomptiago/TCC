/*
 * MovimentationUtils.cpp
 *
 *  Created on: Sep 14, 2015
 *      Author: tiago
 */

#include <controlador_de_trajetoria/utils/MovimentationUtils.h>

bool MovimentationUtils::isMoveRobotEqual(
	controlador_de_trajetoria::Move_robot move1,
	controlador_de_trajetoria::Move_robot move2) {
		return move1.x == move2.x && move1.y == move2.y;
}
