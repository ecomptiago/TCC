/*
 * AStarGridCell.h
 *
 *  Created on: Feb 4, 2016
 *      Author: tcoelho
 */

#ifndef PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_SEARCH_ASTAR_ASTARGRIDCELL_H_
#define PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_SEARCH_ASTAR_ASTARGRIDCELL_H_

#include "path_planner/search/BaseGridCell.h"

class AStarGridCell: public BaseGridCell<float> {

	private:

		//Attributes
		float heutisticValue;

	public:
		float getHeutisticValue();
		void setHeutisticValue(float heutisticValue);

		float calculateCellCost(int position);
};

#endif /* PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_SEARCH_ASTAR_ASTARGRIDCELL_H_ */
