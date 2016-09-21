/*
 * NeuralNetwork.cpp
 *
 *  Created on: Sep 21, 2016
 *      Author: tcoelho
 */

#include "neural_network/NeuralNetwork.h"

NeuralNetwork::NeuralNetwork(int argc, char **argv) :
	BaseRosNode(argc, argv, "NeuralNetwork") {

}

int NeuralNetwork::runNode() {
	return 0;
}

bool NeuralNetwork::subscribeToTopics() {
	return false;
}

bool NeuralNetwork::createPublishers() {
	return false;
}

int main(int argc, char **argv) {
//    fann_type *output;
//    fann_type input[180];
//
//    std::string neuralNetowrkFile = get_current_dir_name();
//    neuralNetowrkFile =
//    	neuralNetowrkFile.erase(neuralNetowrkFile.find("catkin_ws") + 9)
//		.append("/src/neural_network/rna.net");
//
//    struct fann *ann = fann_create_from_file(neuralNetowrkFile.c_str());
//    //adicionar código para conversão de dados do arquivo .txt para um vetor
//
//    output = fann_run(ann, input);
//
//    printf("%f", output[0]);
//    //adicionar código para imprimir vetor output
//
//    fann_destroy(ann);
//
//    return 0;


//	fann_type *calc_out;
//	fann_type input[2];
//
//    std::string neuralNetowrkFile = get_current_dir_name();
//    neuralNetowrkFile =
//    	neuralNetowrkFile.erase(neuralNetowrkFile.find("catkin_ws") + 9)
//		.append("/src/neural_network/xor_float.net");
//
//
//	struct fann *ann = fann_create_from_file(neuralNetowrkFile.c_str());
//
//	input[0] = -1;
//	input[1] = 1;
//	calc_out = fann_run(ann, input);
//
//	printf("xor test (%f,%f) -> %f\n", input[0], input[1], calc_out[0]);
//
//	fann_destroy(ann);
//	return 0;

}
