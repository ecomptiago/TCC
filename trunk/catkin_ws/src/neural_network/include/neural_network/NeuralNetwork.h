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
#include "common/BaseRosNode.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32MultiArray.h"

const char* laserTopic = "/RosAria/laser";
const char* gridTopic = "/NeuralNetwork/grid";

class NeuralNetwork : public BaseRosNode{

	private:
		//Atttributes
		ros::NodeHandle nodeHandler;
		fann_type laserValues[180];
		fann_type *output;
		struct fann *ann;
		int outputSize;

		//Methods
		void destroyNeuralNetwork();

	public:

		//Constructor
		NeuralNetwork(int argc, char **argv);

		//Destructor
		virtual ~NeuralNetwork() {};

		//Methods
		int runNode();
		bool subscribeToTopics();
		bool createPublishers();
		int shutdownAndExit();
		int shutdownAndExit(std::exception &e);
		void receivedLaserValues(
			const sensor_msgs::LaserScan::ConstPtr& laserReading);

};

#endif /* INCLUDE_NEURAL_NETWORK_NEURALNETWORK_H_ */
