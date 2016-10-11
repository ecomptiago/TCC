/*
 * Coordinator.h
 *
 *  Created on: Nov 12, 2015
 *      Author: tcoelho
 */

#ifndef SRC_COORDINATOR_H_
#define SRC_COORDINATOR_H_

#include "algorithm"
#include "vector"
#include "common/BaseRosNode.h"
#include "common/Position.h"
#include "common/pathToTarget.h"
#include "common/cellGridPosition.h"
#include "common/utils/NumericUtils.h"
#include "common/utils/GridUtils.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/OccupancyGrid.h"

const char* laserTopic = "/RosAria/laser";
const char* cmdVelTopic = "/RosAria/cmd_vel";
const char* poseTopic = "/RosAria/pose";
const char* targetPositionTopic = "/MovimentationExecutor/target";
const char*	velTopic = "/MovimentationExecutor/velocity";
const char*	errorTopic = "/MovimentationExecutor/error";
const char* turnAngleTopic = "/AvoidObstacles/turnAngle";
const char* rvizPoseTopic = "/Coordinator/pose";
const char* bestPathService = "/PathPlanner/bestPath";
const char* occupancyGridTopic = "/NeuralNetwork/grid";

class Coordinator : public BaseRosNode{

	private:
		//Atttributes
		ros::NodeHandle nodeHandler;
		std::vector<float> laserValues;
		geometry_msgs::Twist proportionalVelocity;
		std_msgs::Float32 proportionalError;
		geometry_msgs::PoseStamped robotPose;
		std_msgs::Float32 fuzzyTurnAngle;
		bool triedToFindPath;
		bool reachedFinalGoal;
		bool recalculatePath;
		int pathPosition;
		std::vector<common::Position> targetPositions;
		nav_msgs::OccupancyGrid occupancyGrid;

		//Methods
		const common::Position cellGridPosition(int cellGrid);

	public:
		//Constructor
		Coordinator(int argc, char **argv);

		//Destructor
		virtual ~Coordinator() {};

		//Methods
		int runNode();
		bool subscribeToTopics();
		bool createPublishers();
		bool createServices();
		void receivedLaserValues(
			const sensor_msgs::LaserScan::ConstPtr& laserReading);
		void receivedProportionalControlerVelocity(
			const geometry_msgs::Twist::ConstPtr& proportionalVelocity);
		void receivedProportionalControlerError(
			const std_msgs::Float32::ConstPtr& proportionalError);
		void receivedRobotPose(
			const geometry_msgs::PoseStamped::ConstPtr& robotPose);
		void receivedFuzzyTurnAngle(
			const std_msgs::Float32::ConstPtr& fuzzyTurnAngle);
		void receivedOccupancyGrid(
			const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid);

};

#endif /* SRC_COORDINATOR_H_ */
