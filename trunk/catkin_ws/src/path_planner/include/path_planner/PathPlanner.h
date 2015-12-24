/*
 * PathPlanner.h
 *
 *  Created on: Dec 15, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_PATH_PLANNER_PATHPLANNER_H_
#define INCLUDE_PATH_PLANNER_PATHPLANNER_H_

#include "common/BaseRosNode.h"
#include "common/v_repConst.h"
#include "path_planner/simRosGetObjectGroupData.h"

const char* nodeName = "Path_planner";
const char* getObjectGroupDataService = "/vrep/simRosGetObjectGroupData";
const char* wallHandle = "Wall";
const char* cuboidHandle = "Cuboid";

class PathPlanner : public BaseRosNode {

	private:
		//Atttributes
		ros::NodeHandle nodeHandler;

		//Methods
		bool createServiceClients();
		bool createServiceServers();
		int infoFailAndExit(const char* topicName);

	public:
		//Constructor
		PathPlanner(int argc, char **argv);

		//Destructor
		virtual ~PathPlanner() {};

		//Methods
		int runNode();
		bool createServices();
		bool subscribeToTopics();
		bool createPublishers();
};

#endif /* INCLUDE_PATH_PLANNER_PATHPLANNER_H_ */
