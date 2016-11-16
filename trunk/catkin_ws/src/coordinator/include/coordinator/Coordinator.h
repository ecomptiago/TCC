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
#include "common/utils/OdometryUtils.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/OccupancyGrid.h"

const char* laserTopic = "/RosAria/laser";
const char* cmdVelTopic = "/RosAria/cmd_vel";
const char* poseTopic = "/RosAria/pose";
const char* targetPositionTopic = "/MovimentationExecutor/target";
const char*	velTopic = "/MovimentationExecutor/velocity";
const char*	errorTopic = "/MovimentationExecutor/error";
const char* turnAngleTopic = "/AvoidObstacles/turnAngle";
const char* rvizPoseTopic = "/Coordinator/pose";
const char* occupancyGridTopic = "/NeuralNetwork/grid";
const char* updateWorldTopic = "/NeuralNetwork/updateWorld";
const char* bestPathService = "/PathPlanner/bestPath";
const char* targetTopic = "/Coordinator/target";
const char* neuralGridTopic = "/NeuralNetwork/neuralGrid";

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
		std::vector<common::Position> targetPositionsSlam;
		std::vector<common::Position> targetPositionsGuided;
		nav_msgs::OccupancyGrid occupancyGrid;
		std_msgs::Float32MultiArray neuralGrid;
		std::vector<int> cellsInThePath;

		//Methods
		const common::Position cellGridPosition(int cellGrid);
		void moveToCell(int targetCell);
		void turnRobot(int targetAngle);
		bool isFree(int neuralGridCell);
		int getRobotCell();
		void moveForward();
		void stop();
		bool isCellInPath(int robotCell);


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
		void receiveTarget(
			const common::Position::ConstPtr& target);
		void receiveNeuralNetworkGrid(
			const std_msgs::Float32MultiArray::ConstPtr& neuralGrid);

};

#endif /* SRC_COORDINATOR_H_ */
