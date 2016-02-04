/*
 * AStarGridCell.h
 *
 *  Created on: Feb 4, 2016
 *      Author: tcoelho
 */

#ifndef PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_SEARCH_ASTAR_ASTARGRIDCELL_H_
#define PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_SEARCH_ASTAR_ASTARGRIDCELL_H_

#include "math.h"
#include "common/Position.h"
#include "path_planner/search/BaseGridCell.h"

class AStarGridCell: public BaseGridCell<float> {

	private:

		//Attributes
		common::Position targetCoordinates;
		common::Position cellCoordinates;

	public:

		//Constructors
		AStarGridCell();
		AStarGridCell(common::Position cellCoordinates,
			common::Position targetCoordinates, int cellGridPosition);

		//Methods
		bool calculateCellCost(int initialPosition);
		void copy(AStarGridCell &aStarGridCell);
};

#endif /* PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_SEARCH_ASTAR_ASTARGRIDCELL_H_ */
