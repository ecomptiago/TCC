/*
 * MocimentationErrorEnum.h
 *
 *  Created on: Sep 13, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_MOVIMENTATIONERRORENUM_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_MOVIMENTATIONERRORENUM_H_

#include "string"

class MovimentationErrorEnum {
	public:
		enum MovimentationError {
			MOTOR_DISABLED,
			UNKNOW
		};
	std::string getStringFromEnum(MovimentationError enumError);
};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_MOVIMENTATIONERRORENUM_H_ */
