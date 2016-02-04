/*
 * GridCellInterface.h
 *
 *  Created on: Feb 4, 2016
 *      Author: tcoelho
 */

#ifndef PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_SEARCH_GRIDCELLINTERFACE_H_
#define PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_SEARCH_GRIDCELLINTERFACE_H_

class GridCellInterface {

	public:

		//Destructor
		virtual ~GridCellInterface(){};

		virtual bool calculateCellCost(int initialGridPosition) = 0;
};

#endif /* PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_SEARCH_GRIDCELLINTERFACE_H_ */
