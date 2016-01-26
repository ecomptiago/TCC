/*
 * PathPlanner.h
 *
 *  Created on: Dec 15, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_PATH_PLANNER_PATHPLANNER_H_
#define INCLUDE_PATH_PLANNER_PATHPLANNER_H_

#include "vector"
#include "stdio.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Quaternion.h"
#include "common/BaseRosNode.h"
#include "common/v_repConst.h"
#include "common/utils/VRepUtils.h"
#include "common/utils/OdometryUtils.h"
#include "common/utils/NumericUtils.h"
#include "common/utils/MatrixUtils.h"
#include "common/simRosGetObjectPose.h"
#include "common/Position.h"
#include "path_planner/ObjectInfo.h"
#include "path_planner/simRosGetObjectGroupData.h"
#include "path_planner/simRosGetObjectFloatParameter.h"
#include "path_planner/simRosGetObjectChild.h"
#include "path_planner/search/AStar.h"

const char* getObjectGroupDataService = "/vrep/simRosGetObjectGroupData";
const char* getObjectFloatParameterService = "/vrep/simRosGetObjectFloatParameter";
const char* getObjectChildService = "/vrep/simRosGetObjectChild";
const char* mapTopic = "/PathPlanner/map";
const char* cuboidHandle = "Cuboid";
const char* floorHandle = "ResizableFloor_5_25";
const int32_t sim_objfloatparam_modelbbox_min_x = 15;
const int32_t sim_objfloatparam_modelbbox_max_x = 18;
const int32_t sim_objfloatparam_modelbbox_min_y = 16;
const int32_t sim_objfloatparam_modelbbox_max_y = 19;
const int8_t occupiedCell = 100;
const int8_t unknownCell = -1;
const int8_t freeCell = 0;

class PathPlanner : public BaseRosNode {

	private:
		//Atttributes
		ros::NodeHandle nodeHandler;
		std::map<std::string,int32_t> signalObjectMap;
		nav_msgs::OccupancyGrid occupancyGrid;
		float angleTolerance;
		double wakeUpTime;
		AStar aStar;

		//Methods
		bool createServiceClients();
		bool createServiceServers();
		int infoFailAndExit(const char* topicName);
		bool getMinimumXYObjectCoordinate(int32_t objecthandle,
			common::simRosGetObjectPose &simRosGetObjectPose,common::Position &position);
		void callGetFloatParameterService(int32_t objectHandle, int32_t parameterID,
			path_planner::simRosGetObjectFloatParameter &simRosGetObjectFloatParameter);
		bool getObjectWidthHeight(int32_t objectHandle,	path_planner::ObjectInfo &objectInfo);
		int getDataVectorPosition(common::Position &position);
		bool addObjectToOccupancyMap(int32_t childHandle);

	public:
		//Constructor
		PathPlanner(int argc, char **argv, int cellArea, int mapWidth,
			int mapHeight, float angleTolerance, double wakeUpTime);

		//Destructor
		virtual ~PathPlanner() {};

		//Methods
		int runNode();
		bool createServices();
		bool subscribeToTopics();
		bool createPublishers();
};

#endif /* INCLUDE_PATH_PLANNER_PATHPLANNER_H_ */
