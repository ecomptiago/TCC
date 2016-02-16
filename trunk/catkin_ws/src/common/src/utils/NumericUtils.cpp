/*
 * NumericUtils.cpp
 *
 *  Created on: Oct 25, 2015
 *      Author: tiago
 */

#include "../../include/common/utils/NumericUtils.h"

bool NumericUtils::isFirstGreaterEqualWithPrecision(double a, double b,
	int precision) {
		double intpartA;
		double intpartB;
		modf(a*pow10(precision), &intpartA);
		modf(b*pow10(precision), &intpartB);
		return NumericUtils::isFirstGreaterEqual<double>(intpartA,intpartB);
}

bool NumericUtils::isFirstLessEqualWithPrecision(double a, double b,
	int precision) {
		double intpartA;
		double intpartB;
		modf(a*pow10(precision), &intpartA);
		modf(b*pow10(precision), &intpartB);
		return NumericUtils::isFirstLessEqual<double>(intpartA,intpartB);
}

bool NumericUtils::isFirstLessWithPrecision(double a, double b,
	int precision) {
		double intpartA;
		double intpartB;
		modf(a*pow10(precision), &intpartA);
		modf(b*pow10(precision), &intpartB);
		return NumericUtils::isFirstLess<double>(intpartA,intpartB);
}
