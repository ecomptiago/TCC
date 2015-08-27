/*
 * MethodNotImplementedError.cpp
 *
 *  Created on: Aug 27, 2015
 *      Author: tiago
 */

#include "string"
#include  "stdexcept"
#include "ros/ros.h"
#include "controlador_de_trajetoria/error/MethodNotImplementedError.h"

MethodNotImplementedError::MethodNotImplementedError(std::string methodName,
		std::string baseClassName) {
	std::string errorMessage = "A class derivated from " +
			baseClassName + " has to override the method " +
			methodName;
	std::logic_error error(errorMessage);
	ROS_ERROR("Error: %s",error.what());
	throw error;
}

