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
#include "nav_msgs/OccupancyGrid.h"
#include "path_planner/search/BaseGridCell.h"
#include "path_planner/utils/PathPlannerUtils.h"

const int infiniteCost = 1000000;

class AStarGridCell: public BaseGridCell<float> {

	private:

		//Attributes
		common::Position targetCoordinates;
		common::Position cellCoordinates;
		int gCost;
		AStarGridCell *comingFromCell;
		std::vector<AStarGridCell> successors;

		//Methods
		void addNodeToNeighbours(int rightCell,
			nav_msgs::OccupancyGrid& occupancyGrid,
			std::vector<AStarGridCell>& neighbours);

	public:

		//Constructors
		AStarGridCell();
		AStarGridCell(common::Position cellCoordinates,
			common::Position targetCoordinates, int cellGridPosition);

		//Methods
		bool calculateCellCost();
		void copy(AStarGridCell &aStarGridCell);
		void getCellNeighbours(std::vector<AStarGridCell> &neighbours,
			nav_msgs::OccupancyGrid &occupancyGrid);

		//Getters and setters
		int getGCost();
		void setGCost(int gCost);
		AStarGridCell*& getComingFromCell();
		void setComingFromCell(AStarGridCell &comingFromCell);
		std::vector<AStarGridCell>& getSuccessors();
		void setSuccessors(std::vector<AStarGridCell>& successors);
};

#endif /* PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_SEARCH_ASTAR_ASTARGRIDCELL_H_ */
