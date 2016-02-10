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
#include "vector"
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "path_planner/search/SearchAlgorithmInterface.h"
#include "path_planner/search/AStar/AStarGridCell.h"
#include "path_planner/utils/PathPlannerUtils.h"

class AStar: public SearchAlgorithmInterface{

	private:

		//Attributes
		nav_msgs::OccupancyGrid *occupancyGridPointer;
		std::map<int,AStarGridCell> openNodes;
		std::map<int,AStarGridCell> closedNodes;

		//Methods
		void getCellWithSmallerCostOpenNodes(AStarGridCell &aStarGridCell);
		bool successorsContainsCell(std::pair<const int, AStarGridCell> &aStarGridCell, int targetCell);
		void getCellWithSmallerCostAndSuccessor(int targetCell, std::vector<AStarGridCell> &path);

	public:
		//Constructor
		AStar();

		//Destructor
		virtual ~AStar() {};

		//Methods
		virtual bool findPathToGoal(common::Position &initialCoordinates,
			common::Position &targetCoordinates);
		void reconstructPath(std::vector<AStarGridCell> &path, common::Position &targetCoordinates,
			common::Position &initialCoordinates);

		//Getters and setters
		void setOccupancyGrid(nav_msgs::OccupancyGrid &occupancyGrid);
};

#endif /* INCLUDE_PATH_PLANNER_SEARCH_ASTAR_H_ */
