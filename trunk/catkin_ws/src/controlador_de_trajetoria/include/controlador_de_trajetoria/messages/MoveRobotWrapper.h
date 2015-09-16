/*
 * MoveRobotWrapper.h
 *
 *  Created on: Sep 16, 2015
 *      Author: tcoelho
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_MESSAGES_MOVEROBOTWRAPPER_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_MESSAGES_MOVEROBOTWRAPPER_H_

#include "controlador_de_trajetoria/Move_robot.h"
#include "iostream"

static int idCounter;

class MoveRobotWrapper {
	private:
		//Attributes
		int id;
		controlador_de_trajetoria::Move_robot moveRobotObj;

	public:
		//Constructor
		MoveRobotWrapper();
		MoveRobotWrapper(
			const controlador_de_trajetoria::Move_robot::ConstPtr& moveRobotPosition);
		MoveRobotWrapper(
			controlador_de_trajetoria::Move_robot moveRobotPosition);

		//Destructor
		virtual ~MoveRobotWrapper() {};

		//Getters and setters
		int getId() const;
		const controlador_de_trajetoria::Move_robot& getMoveRobotObj() const;

};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_MESSAGES_MOVEROBOTWRAPPER_H_ */
