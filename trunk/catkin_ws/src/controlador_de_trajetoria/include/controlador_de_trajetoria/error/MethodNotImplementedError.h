/*
 * MethodNotImplementedError.h
 *
 *  Created on: Aug 27, 2015
 *      Author: tiago
 */

#ifndef SRC_ERROR_METHODNOTIMPLEMENTEDERROR_H_
#define SRC_ERROR_METHODNOTIMPLEMENTEDERROR_H_

#include "string"

class MethodNotImplementedError {

	public:
		//Constructor
		MethodNotImplementedError(std::string methodName, std::string baseClassName);

		//Destructor
		virtual ~MethodNotImplementedError() {};
};

#endif /* SRC_ERROR_METHODNOTIMPLEMENTEDERROR_H_ */
