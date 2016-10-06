/*
 * Coordinator.h
 *
 *  Created on: Nov 12, 2015
 *      Author: tcoelho
 */

#ifndef SRC_COORDINATOR_H_
#define SRC_COORDINATOR_H_

#include "algorithm"
#include "common/BaseRosNode.h"
#include "common/Position.h"
#include "common/pathToTarget.h"
#include "common/cellGridPosition.h"
#include "common/utils/NumericUtils.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"

const char* laserTopic = "/RosAria/laser";
const char* cmdVelTopic = "/RosAria/cmd_vel";
const char* poseTopic = "/RosAria/pose";
const char* targetPositionTopic = "/MovimentationExecutor/target";
const char*	velTopic = "/MovimentationExecutor/velocity";
const char*	errorTopic = "/MovimentationExecutor/error";
const char* turnAngleTopic = "/AvoidObstacles/turnAngle";
const char* rvizPoseTopic = "/Coordinator/pose";
const char* bestPathService = "/PathPlanner/bestPath";
const char* cellGridPositionService = "/PathPlanner/cellGrid";

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

		//Methods
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
};

#endif /* SRC_COORDINATOR_H_ */
