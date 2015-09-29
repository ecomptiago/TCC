/*
 * MatrixUtils.cpp
 *
 *  Created on: Sep 28, 2015
 *      Author: tcoelho
 */

#include "../../include/common/utils/MatrixUtils.h"

bool MatrixUtils::applyGaussElimeliminationWithPartialPivotingAlgorithm(
	float *linearEquationMatrix, int numberOfEquations, float *response) {
		int i;
		for(i = 0; i < numberOfEquations; i++) {
			int k;
			for(k = 0; k < numberOfEquations + 1; k++) {
				std::cout <<
					getMatrixElement<float>(i, k, linearEquationMatrix, numberOfEquations + 1)
					<< " ";
			}
			std::cout << "\n";
		}
		std::cout << "\n";
		for(i = 0; i < numberOfEquations; i++) {
			std::cout << response[i] << "\t";
		}

//		int lineIndex;
//		for(lineIndex = 0; lineIndex < numberOfEquations -1; lineIndex++) {
//			pivotingColumn(lineIndex, numberOfEquations);
//		}

		return true;

}

void MatrixUtils::pivotingColumn(int lineIndex, int numberOfEquations,
	float *linearEquationMatrix) {
		int count;
		for(count = 0; count < numberOfEquations; count++) {
			int pivotingIndex = lineIndex + count;
//			if(linearEquationMatrix[pivotingIndex])
		}
}

