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
#include "common/utils/NumericUtils.h"
#include "common/Move_robot.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"

const char* laserTopic = "/RosAria/laser";
const char* cmdVelTopic = "/RosAria/cmd_vel";
const char* targetPositionTopic = "/MovimentationExecutor/target";
const char*	velTopic = "/MovimentationExecutor/velocity";
const char*	errorTopic = "/MovimentationExecutor/error";

class Coordinator : public BaseRosNode{

	private:
		//Atttributes
		ros::NodeHandle nodeHandler;
		std::vector<float> laserValues;
		geometry_msgs::Twist proportionalVelocity;
		std_msgs::Float32 proportionalError;
		//Methods

	public:
		//Constructor
		Coordinator(int argc, char **argv);

		//Destructor
		virtual ~Coordinator() {};

		//Methods
		void receivedLaserValues(
			const sensor_msgs::LaserScan::ConstPtr& laserReading);
		int runNode();
		bool subscribeToTopics();
		bool createPublishers();
		void receivedProportionalControlerVelocity(
			const geometry_msgs::Twist::ConstPtr& proportionalVelocity);
		void receivedProportionalControlerError(
			const std_msgs::Float32::ConstPtr& proportionalError);

};

#endif /* SRC_COORDINATOR_H_ */
