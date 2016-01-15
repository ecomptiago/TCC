/*
 * Coordinator.h
 *
 *  Created on: Nov 12, 2015
 *      Author: tcoelho
 */

#ifndef SRC_COORDINATOR_H_
#define SRC_COORDINATOR_H_

#include "algorithm"
#include "iostream"
#include "common/BaseRosNode.h"
#include "common/utils/NumericUtils.h"
#include "sensor_msgs/LaserScan.h"

const char* laserTopic = "/RosAria/laser";

class Coordinator : public BaseRosNode{

	private:
		//Atttributes
		ros::NodeHandle nodeHandler;
		std::vector<float> laserValues;

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
};

#endif /* SRC_COORDINATOR_H_ */
