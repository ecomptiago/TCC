/*
 * AStar.h
 *
 *  Created on: Jan 26, 2016
 *      Author: tcoelho
 */

#ifndef INCLUDE_PATH_PLANNER_SEARCH_ASTAR_H_
#define INCLUDE_PATH_PLANNER_SEARCH_ASTAR_H_

#include "stdio.h"
#include "SearchAlgorithmInterface .h"
#include "nav_msgs/OccupancyGrid.h"
#include "path_planner/utils/PathPlannerUtils.h"

class AStar: public SearchAlgorithmInterface{

	private:
		nav_msgs::OccupancyGrid *occupancyGridPointer;

	public:
		//Constructor
		AStar();

		//Destructor
		virtual ~AStar() {};

		//Methods
		virtual bool findPathToGoal(common::Position &initialPosition,
			common::Position &targetPosition);
		void setOccupancyGrid(nav_msgs::OccupancyGrid &occupancyGrid);
};

#endif /* INCLUDE_PATH_PLANNER_SEARCH_ASTAR_H_ */
