/*
 * AStar.h
 *
 *  Created on: Jan 26, 2016
 *      Author: tcoelho
 */

#ifndef INCLUDE_PATH_PLANNER_SEARCH_ASTAR_H_
#define INCLUDE_PATH_PLANNER_SEARCH_ASTAR_H_

#include "map"
#include "string"
#include "ros/ros.h"
#include "SearchAlgorithmInterface.h"
#include "nav_msgs/OccupancyGrid.h"
#include "path_planner/GridCell.h"
#include "path_planner/utils/PathPlannerUtils.h"

class AStar: public SearchAlgorithmInterface{

	private:
		nav_msgs::OccupancyGrid *occupancyGridPointer;
		std::map<int,GridCell> openNodes;
		std::map<int,GridCell> closedNodes;

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
