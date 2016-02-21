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
		std::vector<AStarGridCell> successors;

		//Methods
		void addNodeToNeighbours(int cellGridPosition,
			nav_msgs::OccupancyGrid& occupancyGrid,
			std::vector<AStarGridCell>& neighbours);
		bool hasRightCell(int cellGridPosition,
			int columns, int rows);
	public:

		//Constructors
		AStarGridCell();
		AStarGridCell(int cellGridPosition);
		AStarGridCell(common::Position &targetCoordinates,
			int cellGridPosition);

		//Methods
		bool calculateCellCost();
		void copy(AStarGridCell &aStarGridCell);
		void getCellNeighbours(std::vector<AStarGridCell> &neighbours,
			nav_msgs::OccupancyGrid &occupancyGrid);
		void calculateCellCoordinates(nav_msgs::OccupancyGrid &occupancyGrid);

		//Getters and setters
		int getGCost();
		void setGCost(int gCost);
		std::vector<AStarGridCell>& getSuccessors();
		void setSuccessors(std::vector<AStarGridCell>& successors);
		common::Position& getTargetCoordinates() ;
		void setTargetCoordinates(common::Position& targetCoordinates);
		common::Position& getCellCoordinates();
};

#endif /* PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_SEARCH_ASTAR_ASTARGRIDCELL_H_ */
