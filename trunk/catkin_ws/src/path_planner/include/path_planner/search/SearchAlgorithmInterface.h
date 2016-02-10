/*
 * SearchAlgorithmInterface .h
 *
 *  Created on: Jan 26, 2016
 *      Author: tcoelho
 */

#ifndef INCLUDE_PATH_PLANNER_SEARCH_SEARCHALGORITHMINTERFACE__H_
#define INCLUDE_PATH_PLANNER_SEARCH_SEARCHALGORITHMINTERFACE__H_

#include "vector"
#include "common/Position.h"
#include "path_planner/search/GridCellInterface.h"

class SearchAlgorithmInterface {
	public:
		//Destructor
		virtual ~SearchAlgorithmInterface () {};

		//Methods
		virtual bool findPathToGoal(common::Position &initialCoordinates,
			common::Position &targetCoordinates) = 0;
};

#endif /* INCLUDE_PATH_PLANNER_SEARCH_SEARCHALGORITHMINTERFACE__H_ */
