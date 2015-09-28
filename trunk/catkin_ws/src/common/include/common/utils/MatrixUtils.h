/*
 * MatrixUtils.h
 *
 *  Created on: Sep 28, 2015
 *      Author: tcoelho
 */

#ifndef SRC_UTILS_MATRIXUTILS_H_
#define SRC_UTILS_MATRIXUTILS_H_

class MatrixUtils {
	public:
		//Constructor
		MatrixUtils() {};

		//Destructor
		virtual ~MatrixUtils() {};

		//Methods
		static bool applyGaussJordanAlgorithm(float *linearEquationMatrix,
			int numberOfVariables, int numberOfEquations, float *response);

};

#endif /* SRC_UTILS_MATRIXUTILS_H_ */
