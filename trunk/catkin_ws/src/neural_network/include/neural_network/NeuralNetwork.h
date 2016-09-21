/*
 * NeuralNetwork.h
 *
 *  Created on: Sep 21, 2016
 *      Author: tcoelho
 */

#ifndef INCLUDE_NEURAL_NETWORK_NEURALNETWORK_H_
#define INCLUDE_NEURAL_NETWORK_NEURALNETWORK_H_

#include "common/BaseRosNode.h"
#include "floatfann.h"
#include "fann.h"
#include "unistd.h"
#include "string.h"

class NeuralNetwork : public BaseRosNode{

	private:

	public:

		//Constructor
		NeuralNetwork(int argc, char **argv);

		//Destructor
		virtual ~NeuralNetwork() {};

		//Methods
		int runNode();
		bool subscribeToTopics();
		bool createPublishers();

};

#endif /* INCLUDE_NEURAL_NETWORK_NEURALNETWORK_H_ */
