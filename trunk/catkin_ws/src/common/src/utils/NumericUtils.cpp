/*
 * NumericUtils.cpp
 *
 *  Created on: Oct 25, 2015
 *      Author: tiago
 */

#include "../../include/common/utils/NumericUtils.h"

double NumericUtils::round(double number) {
	double fractpart;
	double intpart;
	fractpart = modf (number , &intpart);
	if(fractpart < 0.51) {
		return intpart;
	} else {
		return intpart + 1;
	}
}
