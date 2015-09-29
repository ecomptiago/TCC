/*
 * MatrixUtils.h
 *
 *  Created on: Sep 28, 2015
 *      Author: tcoelho
 */

#ifndef SRC_UTILS_MATRIXUTILS_H_
#define SRC_UTILS_MATRIXUTILS_H_

#include "iostream"
#include "math.h"

class MatrixUtils {

	private:
		//Methods

		/*Methods that need to be implemented here, because of compiling
		 *issues. More information at
		 *http://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor
		 */
		template<class T>
		static T getMatrixElement(int lineIndex, int columnIndex,
			T *linearEquationMatrix, int columnNumber) {
				return linearEquationMatrix[columnNumber * lineIndex + columnIndex];
		}

		template<class T>
		static void setMatrixElement(int lineIndex, int columnIndex,
			T *linearEquationMatrix, int columnNumber, T value) {
				linearEquationMatrix[columnNumber * lineIndex + columnIndex] =  value;
		}

		template<class T>
		static void swapLine(int lineIndex, int lineIndex2, int numberOfEquations,
			T *linearEquationMatrix, int columnNumber) {
			T aux;
			int count;
			for(count = 0; count < columnNumber; count++) {
				aux =  getMatrixElement(lineIndex, count, linearEquationMatrix,columnNumber);
				setMatrixElement(lineIndex, count, linearEquationMatrix,columnNumber,
					getMatrixElement(lineIndex2, count, linearEquationMatrix,columnNumber));
				setMatrixElement(lineIndex2, count, linearEquationMatrix,columnNumber,aux);
			}
		}

		template<class T>
		static void pivotingColumn(int lineIndex, int numberOfEquations,
			T *linearEquationMatrix) {
				int count;
				for(count = 1; count < numberOfEquations; count++) {
					int pivotingIndex = lineIndex + count;
					T pivotingElement =
						getMatrixElement<T>(pivotingIndex, lineIndex, linearEquationMatrix,
						numberOfEquations + 1);
					T lineElement =
						getMatrixElement<T>(lineIndex, lineIndex, linearEquationMatrix,
						numberOfEquations + 1);
					if(pivotingElement != 0) {
						if(lineElement == 0 || mod(pivotingElement) > mod(lineElement)) {
							swapLine(lineIndex, pivotingIndex, numberOfEquations,
								linearEquationMatrix, numberOfEquations + 1);
						}
					}
				}
		}

		template<class T>
		static float mod(T number) {
			if(number < 0) {
				return number * -1;
			} else {
				return number;
			}
		}

		template<class T>
		static void variablesElimination(int lineIndex, int numberOfEquations,
			T *linearEquationMatrix) {
				int substitutionLineIndex;
				for(substitutionLineIndex = lineIndex + 1; substitutionLineIndex < numberOfEquations;
					substitutionLineIndex++) {
						float m =
							getMatrixElement(substitutionLineIndex, lineIndex, linearEquationMatrix,
							numberOfEquations + 1) / getMatrixElement(lineIndex, lineIndex,
							linearEquationMatrix, numberOfEquations + 1);
						multiplyByConstantAndSubtract<T>(substitutionLineIndex, lineIndex, m,
							linearEquationMatrix, numberOfEquations + 1);
				}
		}

		template<class T>
		static void multiplyByConstantAndSubtract(int substitutionLineIndex, int lineIndex, float m,
			T *linearEquationMatrix, int columnNumber) {
			int count;
			for(count = 0; count < columnNumber; count++) {
				T result =
					getMatrixElement(substitutionLineIndex, count, linearEquationMatrix, columnNumber)
					 -  m * getMatrixElement(lineIndex, count, linearEquationMatrix, columnNumber);
				setMatrixElement(substitutionLineIndex, count, linearEquationMatrix, columnNumber,
					result);
			}
		}

		template<class T>
		static void retroativeSubstitution(T *linearEquationMatrix, T *response, int numberOfEquations) {
			int responseIndex = numberOfEquations - 1;
			response[responseIndex] =
				getMatrixElement(responseIndex, numberOfEquations, linearEquationMatrix,
				numberOfEquations + 1) / getMatrixElement(responseIndex, responseIndex,
				linearEquationMatrix, numberOfEquations + 1);
			for(responseIndex = responseIndex - 1; responseIndex >= 0; responseIndex--) {
				T sum = 0;
				int sumIndex;
				for(sumIndex = responseIndex + 1; sumIndex < numberOfEquations; sumIndex++) {
					sum = sum + response[sumIndex] * getMatrixElement(responseIndex, sumIndex,
					linearEquationMatrix, numberOfEquations + 1);
				}
				response[responseIndex] = (getMatrixElement(responseIndex, numberOfEquations,
					linearEquationMatrix, numberOfEquations + 1) - sum) / getMatrixElement(responseIndex,
					responseIndex, linearEquationMatrix, numberOfEquations + 1);
			}
		}

	public:
		//Constructor
		MatrixUtils() {};

		//Destructor
		virtual ~MatrixUtils() {};

		//Methods
		template<class T>
		static bool applyGaussElimeliminationWithPartialPivotingAlgorithm(
			T *linearEquationMatrix, int numberOfEquations, T *response) {
				int lineIndex = 0;
				for(lineIndex = 0; lineIndex < numberOfEquations -1; lineIndex++) {
					pivotingColumn<T>(lineIndex, numberOfEquations, linearEquationMatrix);
					if(getMatrixElement<T>(lineIndex, lineIndex, linearEquationMatrix,
						numberOfEquations + 1) == 0) {
							return false;
					} else {
						variablesElimination<T>(lineIndex, numberOfEquations,
							linearEquationMatrix);
					}
				}
				retroativeSubstitution<T>(linearEquationMatrix, response, numberOfEquations);
				return true;
		}


};

#endif /* SRC_UTILS_MATRIXUTILS_H_ */
