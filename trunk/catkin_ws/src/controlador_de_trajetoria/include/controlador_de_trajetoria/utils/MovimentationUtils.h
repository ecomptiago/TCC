/*
 * MovimentationUtils.h
 *
 *  Created on: Sep 14, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_UTILS_MOVIMENTATIONUTILS_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_UTILS_MOVIMENTATIONUTILS_H_

#include "controlador_de_trajetoria/Move_robot.h"

class MovimentationUtils {
	public:
		//Constructor
		MovimentationUtils() {};

		//Destructor
		virtual ~MovimentationUtils() {};

		//Methods
		static bool isMoveRobotEqual(
			controlador_de_trajetoria::Move_robot move1,
			controlador_de_trajetoria::Move_robot move2);
};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_UTILS_MOVIMENTATIONUTILS_H_ */
