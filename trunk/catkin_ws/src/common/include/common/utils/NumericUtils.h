/*
 * NumericUtils.h
 *
 *  Created on: Oct 25, 2015
 *      Author: tiago
 */

#ifndef SRC_UTILS_NUMERICUTILS_H_
#define SRC_UTILS_NUMERICUTILS_H_

#include "functional"

class NumericUtils {

	public:
		//Constructor
		NumericUtils() {};

		//Destructor
		virtual ~NumericUtils() {};

		//Methods
		template<class T>
		static bool isFirstGreater(T a, T b) {
			return std::greater<T>().operator()(a,b);
		}

		template<class T>
		static bool isFirstGreaterEqual(T a, T b) {
			return std::greater_equal<T>().operator()(a,b);
		}

		template<class T>
		static bool isFirstSecondGreater(T a, T b, T c) {
			return std::greater<T>().operator()(a,c) &&
				std::greater<T>().operator()(b,c);
		}

		template<class T>
		static bool isFirstLess(T a, T b) {
			return std::less<T>().operator()(a,b);
		}

		template<class T>
		static bool isFirstLessEqual(T a, T b) {
			return std::less_equal<T>().operator()(a,b);
		}

		template<class T>
		static bool isFirstSecondLessEqual(T a, T b, T c) {
			return std::less_equal<T>().operator()(a,c) &&
				std::less_equal<T>().operator()(b,c);
		}

		template<class T>
		static T mod(T number) {
			if(number < 0) {
				return number * -1;
			} else {
				return number;
			}
		}

};

#endif /* SRC_UTILS_NUMERICUTILS_H_ */
