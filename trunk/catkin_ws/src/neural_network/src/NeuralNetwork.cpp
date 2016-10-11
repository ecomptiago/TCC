/*
 * NeuralNetwork.cpp
 *
 *  Created on: Sep 21, 2016
 *      Author: tcoelho
 */

#include "neural_network/NeuralNetwork.h"

NeuralNetwork::NeuralNetwork(int argc, char **argv, int cellArea,
	int mapWidth, int mapHeight) : BaseRosNode(argc, argv, "NeuralNetwork") {
	    std::string neuralNetowrkFile = get_current_dir_name();
	    neuralNetowrkFile =
	    	neuralNetowrkFile.erase(neuralNetowrkFile.find("catkin_ws") + 9)
			.append("/src/neural_network/rna.net");
		ann = fann_create_from_file(neuralNetowrkFile.c_str());
		output = NULL;
		outputSize= fann_get_num_output(ann);
		this->occupancyGrid.info.resolution = cellArea;
		this->occupancyGrid.info.width = mapWidth;
		this->occupancyGrid.info.height = mapHeight;
		this->occupancyGrid.header.frame_id = "LaserScannerBody_2D";
		for(int i = 0; i < (mapWidth * mapHeight) / cellArea; i++) {
			this->occupancyGrid.data.insert(this->occupancyGrid.data.begin(),unknownCell);
		}

}

int NeuralNetwork::runNode() {
	ROS_INFO("Running node");
    ros::Rate rate(1/0.5);

	neural_network::simRosGetObjectChild simRosGetObjectChild;
	bool firstLoop = true;
	int index = 0;


	//Getting origin coordinates
	if(VRepUtils::getObjectHandle(floorHandle,nodeHandler,signalObjectMap)) {
		geometry_msgs::PoseStamped origin;
		do{
			simRosGetObjectChild.request.handle = signalObjectMap[floorHandle];
			simRosGetObjectChild.request.index = index;
			serviceClientsMap[getObjectChildService].call(simRosGetObjectChild);
			int32_t childHandle = simRosGetObjectChild.response.childHandle;
			if (childHandle != responseError) {
				common::simRosGetObjectPose simRosGetObjectPose;
				if(VRepUtils::getObjectPose(simRosGetObjectChild.response.childHandle,
					nodeHandler,simRosGetObjectPose)) {
						if(firstLoop || (
							simRosGetObjectPose.response.pose.pose.position.x < origin.pose.position.x &&
							simRosGetObjectPose.response.pose.pose.position.y < origin.pose.position.y)) {
								origin = simRosGetObjectPose.response.pose;
								firstLoop = false;
						}
				} else {
					ROS_ERROR("Could not get pose of a children from object %s",floorHandle);
					return shutdownAndExit();
				}
			}
			index++;
		} while(simRosGetObjectChild.response.childHandle != responseError);
		occupancyGrid.info.origin.orientation = origin.pose.orientation;
		occupancyGrid.info.origin.position.x = origin.pose.position.x - 2.5;
		occupancyGrid.info.origin.position.y = origin.pose.position.y - 2.5;
	} else {
		ROS_ERROR("Could not found handle for object %s",floorHandle);
		return shutdownAndExit();
	}

//	occupancyGrid.data[20] = occupiedCell;
//	occupancyGrid.data[21] = occupiedCell;
//	occupancyGrid.data[22] = occupiedCell;
//	occupancyGrid.data[23] = occupiedCell;
//	occupancyGrid.data[24] = occupiedCell;
//
//	occupancyGrid.data[43] = occupiedCell;
//	occupancyGrid.data[44] = occupiedCell;
//	occupancyGrid.data[45] = occupiedCell;
//	occupancyGrid.data[46] = occupiedCell;
//	occupancyGrid.data[47] = occupiedCell;
//
//	occupancyGrid.data[71] = occupiedCell;
//	occupancyGrid.data[72] = occupiedCell;
//	occupancyGrid.data[73] = occupiedCell;
//	occupancyGrid.data[74] = occupiedCell;
//	occupancyGrid.data[75] = occupiedCell;

	while(ros::ok()) {
		sleepAndSpin(rate);
	    output = fann_run(ann, laserValues);

	    double angle = OdometryUtils::getAngleFromQuaternation(
   	 		tf::Quaternion(0,0,
   			robotPose.pose.orientation.z,
   			robotPose.pose.orientation.w),false);
	     		if(NumericUtils::isFirstLess<float>(angle,0.0)) {
	     			angle = angle + 360;
	     		}

	     		ROS_DEBUG("Angle %f",angle);
	     		if(NumericUtils::isFirstGreaterEqual<float>(angle,80) &&
	     			NumericUtils::isFirstLessEqual<float>(angle, 100)) {
	     				angle = 90;
	     		} else if(NumericUtils::isFirstGreaterEqual<float>(angle,170) &&
	     			NumericUtils::isFirstLessEqual<float>(angle, 190)) {
	     				angle = 180;
	     		} else if(NumericUtils::isFirstGreaterEqual<float>(angle,260) &&
	     	 		NumericUtils::isFirstLessEqual<float>(angle, 280)) {
	     			 angle = 270;
	     		} else if((NumericUtils::isFirstGreaterEqual<float>(angle,0) &&
	     			NumericUtils::isFirstLessEqual<float>(angle, 10)) ||
	     			(NumericUtils::isFirstGreaterEqual<float>(angle,350) &&
	     			NumericUtils::isFirstLessEqual<float>(angle, 360))) {
	     				angle = 0;
	     		}

	     		angle = (angle * M_PI) / 180;

	     		int i = 0;

	     		for(double u = 1; u < 8; u++ ) {
	     			for(double v = 3; v > -4; v--) {
	     				float x = (u * cos(angle)) - (v * sin(angle));
	     				float y = (u * sin(angle)) + (v * cos(angle));
	     				x = x + robotPose.pose.position.x;
	     				y = y + robotPose.pose.position.y;
	     				float cellValue;
	     				common::Position position;
	     				position.x = x;
	     				position.y = y;
	     				int cellPosition =
	     					GridUtils::getDataVectorPosition(occupancyGrid,position);
	    // 				ROS_DEBUG("Position.x %f , Position.y %f , cellPosition %d", x, y, cellPosition);
	     				if(cellPosition != -1) {
	    					if(NumericUtils::isFirstLess<float>(output[i],0.0)) {
	    						cellValue = output[i] * -1;
	    					} else {
	    						cellValue = output[i];
	    					}
	    					if(NumericUtils::isFirstLessEqual<float>(cellValue, 0.5)) {
	    						occupancyGrid.data[cellPosition] = freeCell;
	    					} else {
	    						occupancyGrid.data[cellPosition] = occupiedCell;
	    					}
	     				}
	    				i++;
	     			}
	     		}

		publisherMap[occupancyGridTopic].publish(occupancyGrid);
	}
	return shutdownAndExit();
}

bool NeuralNetwork::subscribeToTopics() {
	ROS_INFO("Subscribing to topics");
	return addSubscribedTopic<const sensor_msgs::LaserScan::ConstPtr&, NeuralNetwork>(nodeHandler,laserTopic,
		&NeuralNetwork::receivedLaserValues,this);
}

bool NeuralNetwork::createPublishers() {
	ROS_INFO("Creating publishers");
	return addPublisherClient<nav_msgs::OccupancyGrid>(
		nodeHandler, occupancyGridTopic, false);
}

void NeuralNetwork::destroyNeuralNetwork() {
	if (ann != NULL) {
		fann_destroy(ann);
	}
}

int NeuralNetwork::shutdownAndExit() {
	destroyNeuralNetwork();
	return BaseRosNode::shutdownAndExit();
}

int NeuralNetwork::shutdownAndExit(std::exception& e) {
	destroyNeuralNetwork();
	return BaseRosNode::shutdownAndExit(e);
}

bool NeuralNetwork::createServiceClients() {
	return addServiceClient<neural_network::simRosGetObjectGroupData>(nodeHandler, getObjectGroupDataService) &&
		   addServiceClient<neural_network::simRosGetObjectFloatParameter>(nodeHandler, getObjectFloatParameterService) &&
		   addServiceClient<neural_network::simRosGetObjectChild>(nodeHandler,getObjectChildService);
}

bool NeuralNetwork::createServiceServers() {
	return true;
}

bool NeuralNetwork::createServices() {
	return createServiceClients() && createServiceServers();
}

//Callback
void NeuralNetwork::receivedLaserValues(
	const sensor_msgs::LaserScan::ConstPtr& laserReading) {
		for(int i = 0; i < laserReading->ranges.capacity();i++) {
			laserValues[i] = laserReading->ranges[i];
		}
}

void NeuralNetwork::receivedRobotPose(const geometry_msgs::PoseStamped::ConstPtr& robotPose){
	this->robotPose.header = robotPose->header;
	this->robotPose.pose = robotPose->pose;
}


//Para treinar e configurar a rede devo comentar as linhas 77 a 87! Isto evita publicações no ROS.
int main(int argc, char **argv) {

	NeuralNetwork neuralNetwork(argc,argv,1,10,10);
	try{
		if(neuralNetwork.subscribeToTopics() &&
			neuralNetwork.createPublishers() &&
			neuralNetwork.createServices()) {
				return neuralNetwork.runNode();
		} else {
			 return neuralNetwork.shutdownAndExit();
		}
	} catch (std::exception &e) {
		return neuralNetwork.shutdownAndExit(e);
	}



//    fann_type *output;
//    fann_type input[180];

//    std::string neuralNetowrkFile = get_current_dir_name();
//    neuralNetowrkFile =
//    	neuralNetowrkFile.erase(neuralNetowrkFile.find("catkin_ws") + 9)
//		.append("/src/neural_network/rna.net");
//
//    struct fann *ann = fann_create_from_file("/home/tcoelho/rna2.net");
    //adicionar código para conversão de dados do arquivo .txt para um vetor

//    output = fann_run(ann, input);
//
//	int outputLength = fann_get_num_output(ann);
//
//	for(int i = 0; i < outputLength; i++) {
//		printf("%f\n", output[i]);
//	}

//    printf("%f", output[0]);
//    adicionar código para imprimir vetor output

//    fann_destroy(ann);

//    return 0;


//    const unsigned int num_input = 180;
//    const unsigned int num_output = 49;
//    const unsigned int num_layers = 3;
//    const unsigned int num_neurons_hidden = 361;
//    const float desired_error = (const float) 0.003;
//    const unsigned int max_epochs = 500000;
//    const unsigned int epochs_between_reports = 1000;
//
//    struct fann *ann = fann_create_standard(num_layers, num_input, num_neurons_hidden, num_output);
//
//    fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
//    fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);
//
//    fann_train_on_file(ann, "/home/tcoelho/rna.data", max_epochs, epochs_between_reports, desired_error);
//
//    fann_save(ann, "/home/tcoelho/rna2.net");
//
//    fann_destroy(ann);
//
//    return 0;

//	fann_type *calc_out;
//	fann_type input[2];

//    std::string neuralNetowrkFile = get_current_dir_name();
//    neuralNetowrkFile =
//    	neuralNetowrkFile.erase(neuralNetowrkFile.find("catkin_ws") + 9)
//		.append("/src/neural_network/xor_float.net");


//	struct fann *ann = fann_create_from_file("/home/tcoelho/fann/examples/xor_float.net");
//
//	input[0] = -1;
//	input[1] = 1;
//	calc_out = fann_run(ann, input);
//
//	int output = fann_get_num_output(ann);
//
//	for(int i = 0; i < output; i++) {
//		printf("%f\n", calc_out[i]);
//	}


//	printf("xor test (%f,%f) -> %f\n", input[0], input[1], calc_out[0]);

//	fann_destroy(ann);
//	return 0;

//	fann_type *calc_out;
//		const unsigned int num_input = 2;
//		const unsigned int num_output = 1;
//		const unsigned int num_layers = 3;
//		const unsigned int num_neurons_hidden = 3;
//		const float desired_error = (const float) 0;
//		const unsigned int max_epochs = 1000;
//		const unsigned int epochs_between_reports = 10;
//		struct fann *ann;
//		struct fann_train_data *data;
//
//		unsigned int i = 0;
//		unsigned int decimal_point;
//
//		printf("Creating network.\n");
//		ann = fann_create_standard(num_layers, num_input, num_neurons_hidden, num_output);
//
//		data = fann_read_train_from_file("/home/tcoelho/fann/examples/xor.data");
//
//		fann_set_activation_steepness_hidden(ann, 1);
//		fann_set_activation_steepness_output(ann, 1);
//
//		fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
//		fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);
//
//		fann_set_train_stop_function(ann, FANN_STOPFUNC_BIT);
//		fann_set_bit_fail_limit(ann, 0.01f);
//
//		fann_set_training_algorithm(ann, FANN_TRAIN_RPROP);
//
//		fann_init_weights(ann, data);
//
//		printf("Training network.\n");
//		fann_train_on_data(ann, data, max_epochs, epochs_between_reports, desired_error);
//
//		printf("Testing network. %f\n", fann_test_data(ann, data));
//
//		for(i = 0; i < fann_length_train_data(data); i++)
//		{
//			calc_out = fann_run(ann, data->input[i]);
//			printf("XOR test (%f,%f) -> %f, should be %f, difference=%f\n",
//				   data->input[i][0], data->input[i][1], calc_out[0], data->output[i][0],
//				   fann_abs(calc_out[0] - data->output[i][0]));
//		}
//
//		printf("Saving network.\n");
//
//		fann_save(ann, "/home/tcoelho/fann/examples/xor_float.net");
//
//		decimal_point = fann_save_to_fixed(ann, "xor_fixed.net");
//		fann_save_train_to_fixed(data, "xor_fixed.data", decimal_point);
//
//		printf("Cleaning up.\n");
//		fann_destroy_train(data);
//		fann_destroy(ann);
//
//		return 0;

}
