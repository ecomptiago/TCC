/*
 * MatrixUtils.h
 *
 *  Created on: Sep 28, 2015
 *      Author: tcoelho
 */

#ifndef SRC_UTILS_MATRIXUTILS_H_
#define SRC_UTILS_MATRIXUTILS_H_

#include "iostream"

class MatrixUtils {

	private:
		//Methods
		void pivotingColumn(int lineIndex, int numberOfEquations,
			float *linearEquationMatrix);

		/*Methods that need to be implemented here, because of compiling
		 *issues. More information at
		 *http://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor
		 */
		template<class T>
		static T getMatrixElement(int lineIndex, int columnIndex,
			T *linearEquationMatrix, int columnNumber) {
				return linearEquationMatrix[columnNumber * lineIndex + columnIndex];
		}

	public:
		//Constructor
		MatrixUtils() {};

		//Destructor
		virtual ~MatrixUtils() {};

		//Methods
		static bool applyGaussElimeliminationWithPartialPivotingAlgorithm(
			float *linearEquationMatrix, int numberOfEquations, float *response);

};

#endif /* SRC_UTILS_MATRIXUTILS_H_ */
