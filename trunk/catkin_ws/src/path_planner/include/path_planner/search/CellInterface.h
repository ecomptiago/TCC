/*
 * CellInterface.h
 *
 *  Created on: Jan 28, 2016
 *      Author: tcoelho
 */

#ifndef PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_SEARCH_CELLINTERFACE_H_
#define PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_SEARCH_CELLINTERFACE_H_

class CellInterface {

	public:

		//Destructor
		virtual ~CellInterface() {};

		//Methods
		virtual bool calculateHeuristicValue(int targetPosition) = 0;

};

#endif /* PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_SEARCH_CELLINTERFACE_H_ */
