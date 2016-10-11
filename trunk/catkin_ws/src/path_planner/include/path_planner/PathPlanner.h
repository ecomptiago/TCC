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
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "common/BaseRosNode.h"
#include "common/v_repConst.h"
#include "common/utils/VRepUtils.h"
#include "common/utils/OdometryUtils.h"
#include "common/utils/MatrixUtils.h"
#include "common/utils/GridUtils.h"
#include "common/simRosGetObjectPose.h"
#include "common/Position.h"
#include "common/pathToTarget.h"
#include "path_planner/search/AStar/AStar.h"

const char* poseTopic = "/RosAria/pose";
const char* occupancyGridTopic = "/NeuralNetwork/grid";
const char* bestPathService = "/PathPlanner/bestPath";
const char* pionnerHandle = "Pionner_LX";
const int32_t sim_objfloatparam_modelbbox_min_x = 15;
const int32_t sim_objfloatparam_modelbbox_max_x = 18;
const int32_t sim_objfloatparam_modelbbox_min_y = 16;
const int32_t sim_objfloatparam_modelbbox_max_y = 19;

class PathPlanner : public BaseRosNode {

	private:
		//Atttributes
		ros::NodeHandle nodeHandler;
		std::map<std::string,int32_t> signalObjectMap;
		nav_msgs::OccupancyGrid occupancyGrid;
		AStar aStar;
		geometry_msgs::PoseStamped robotPose;

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
		void receivedRobotPose(
			const geometry_msgs::PoseStamped::ConstPtr& robotPose);
		void receivedOccupancyGrid(
			const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid);
		bool bestPath(common::pathToTarget::Request  &req,
			common::pathToTarget::Response &res);

};

#endif /* INCLUDE_PATH_PLANNER_PATHPLANNER_H_ */
