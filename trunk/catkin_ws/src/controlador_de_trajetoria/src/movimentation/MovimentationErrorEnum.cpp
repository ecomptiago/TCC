/*
 * MovimentationErrorEnum.cpp
 *
 *  Created on: Sep 13, 2015
 *      Author: tiago
 */

#include "controlador_de_trajetoria/movimentation/MovimentationErrorEnum.h"

std::string MovimentationErrorEnum::getStringFromEnum(MovimentationError enumError) {
	switch(enumError) {
		case MovimentationErrorEnum::MOTOR_DISABLED:
			return "Motor was disabled. Trying to enable it again";
		default:
			return "Unknown error";
	}
}


