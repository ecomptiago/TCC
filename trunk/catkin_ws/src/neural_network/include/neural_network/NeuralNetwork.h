/*
 * NeuralNetwork.h
 *
 *  Created on: Sep 21, 2016
 *      Author: tcoelho
 */

#ifndef INCLUDE_NEURAL_NETWORK_NEURALNETWORK_H_
#define INCLUDE_NEURAL_NETWORK_NEURALNETWORK_H_

#include "floatfann.h"
#include "fann.h"
#include "unistd.h"
#include "string.h"
#include "fann_train.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include "common/Position.h"
#include "common/BaseRosNode.h"
#include "common/utils/VRepUtils.h"
#include "common/utils/OdometryUtils.h"
#include "common/utils/NumericUtils.h"
#include "common/utils/GridUtils.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "neural_network/simRosGetObjectGroupData.h"
#include "neural_network/simRosGetObjectFloatParameter.h"
#include "neural_network/simRosGetObjectChild.h"

const char* getObjectGroupDataService = "/vrep/simRosGetObjectGroupData";
const char* getObjectFloatParameterService = "/vrep/simRosGetObjectFloatParameter";
const char* getObjectChildService = "/vrep/simRosGetObjectChild";
const char* laserTopic = "/RosAria/laser";
const char* occupancyGridTopic = "/NeuralNetwork/grid";
const char* floorHandle = "ResizableFloor_5_25";
const char* pionnerHandle = "Pionner_LX";
const int32_t sim_objfloatparam_modelbbox_min_x = 15;
const int32_t sim_objfloatparam_modelbbox_max_x = 18;
const int32_t sim_objfloatparam_modelbbox_min_y = 16;
const int32_t sim_objfloatparam_modelbbox_max_y = 19;
const int8_t occupiedCell = 100;
const int8_t unknownCell = -1;
const int8_t freeCell = 0;


class NeuralNetwork : public BaseRosNode{

	private:
		//Atttributes
		ros::NodeHandle nodeHandler;
		fann_type laserValues[180];
		fann_type *output;
		struct fann *ann;
		int outputSize;
		nav_msgs::OccupancyGrid occupancyGrid;
		std::map<std::string,int32_t> signalObjectMap;
		geometry_msgs::PoseStamped robotPose;

		//Methods
		void destroyNeuralNetwork();
		bool createServiceClients();
		bool createServiceServers();

	public:

		//Constructor
		NeuralNetwork(int argc, char **argv, int cellArea,
			int mapWidth, int mapHeight);

		//Destructor
		virtual ~NeuralNetwork() {};

		//Methods
		int runNode();
		bool subscribeToTopics();
		bool createPublishers();
		bool createServices();
		int shutdownAndExit();
		int shutdownAndExit(std::exception &e);
		void receivedLaserValues(
			const sensor_msgs::LaserScan::ConstPtr& laserReading);
		void receivedRobotPose(
			const geometry_msgs::PoseStamped::ConstPtr& robotPose);

};

#endif /* INCLUDE_NEURAL_NETWORK_NEURALNETWORK_H_ */
