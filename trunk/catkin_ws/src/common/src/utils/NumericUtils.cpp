/*
 * NumericUtils.cpp
 *
 *  Created on: Oct 25, 2015
 *      Author: tiago
 */

#include "../../include/common/utils/NumericUtils.h"

double NumericUtils::round(double number, double middleValue) {
	double fractpart;
	double intpart;
	fractpart = modf (number , &intpart);
	if(fractpart < 0.6) {
		return intpart;
	} else {
		return intpart + 1;
	}
}

double NumericUtils::round(double number) {
	return round(number,0.5);
}
