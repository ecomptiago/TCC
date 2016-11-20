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
		updateWorld = true;
		for(int i = 0; i <100; i++) {
			freeCellVector[i] = 0;
			occupiedCellVector[i] = 0;
			visitedCell[i] = 0;
		}
}

int NeuralNetwork::runNode() {

	ROS_INFO("Running node");

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

	while(ros::ok()) {
		sleepAndSpin(500);
		if(updateWorld) {
			output = fann_run(ann, laserValues);

			double angle = OdometryUtils::getAngleFromQuaternation(
				tf::Quaternion(0,0,
				robotPose.pose.orientation.z,
				robotPose.pose.orientation.w),false);
			if(NumericUtils::isFirstLess<float>(angle,0.0)) {
				angle = angle + 360;
			}

			fillWorldgrid(angle);

//			std_msgs::Float32MultiArray grid;
//			grid.data.resize(outputSize);
//			for(int i = 0; i < outputSize ; i++) {
//				grid.data[i] = output[i];
//			}
//			publisherMap[neuralGridTopic].publish(grid);
	    }
		publisherMap[occupancyGridTopic].publish(occupancyGrid);
	}
	return shutdownAndExit();
}

void NeuralNetwork::fillWorldgrid(double angle) {
	int i = 0;
	angle = (angle * M_PI) / 180;
	for(int v = -1; v < 2; v++) {
		for(int u = 0 ;u < 3; u++ ) {
			if(u != 0 || v != 0) {
				float x = ((u) * cos(angle)) - ((v) * sin(angle));
				float y = ((u) * sin(angle)) + ((v) * cos(angle));
				x = x + robotPose.pose.position.x;
				y = y + robotPose.pose.position.y;
				common::Position position;
				position.x = x;
				position.y = y;
				int cellPosition =
					GridUtils::getDataVectorPosition(occupancyGrid,position);
						ROS_DEBUG("Position.x %f , Position.y %f , cellPosition %d, neuralCellValue %f, "
							"output %d", x, y, cellPosition,output[i], i);
				if(cellPosition != -1) {
					if(NumericUtils::isFirstLessEqual<float>(output[i], 0.7)) {
						freeCellVector[cellPosition]=freeCellVector[cellPosition]+1;
					} else {
						occupiedCellVector[cellPosition]++;
					}
					float i;
					if(occupiedCellVector[cellPosition] != 0) {
						i = freeCellVector[cellPosition] / occupiedCellVector[cellPosition];
					} else {
						i = freeCellVector[cellPosition];
					}

					if(NumericUtils::isFirstGreaterEqual<float>(i,1)) {
						occupancyGrid.data[cellPosition] = freeCell;
					} else if(visitedCell[cellPosition] != 1){
						occupancyGrid.data[cellPosition] = occupiedCell;
					}
				}
				i++;
			} else {
				float x = ((u) * cos(angle)) - ((v) * sin(angle));
				float y = ((u) * sin(angle)) + ((v) * cos(angle));
				x = x + robotPose.pose.position.x;
				y = y + robotPose.pose.position.y;
				common::Position position;
				position.x = x;
				position.y = y;
				int cellPosition =
					GridUtils::getDataVectorPosition(occupancyGrid,position);
				if(cellPosition != -1) {
					visitedCell[cellPosition] = 1;
					freeCellVector[cellPosition]++;
					occupancyGrid.data[cellPosition] = freeCell;
				}
			}

		}
	}
}

bool NeuralNetwork::subscribeToTopics() {
	ROS_INFO("Subscribing to topics");
	return addSubscribedTopic<const sensor_msgs::LaserScan::ConstPtr&, NeuralNetwork>(nodeHandler,laserTopic,
		&NeuralNetwork::receivedLaserValues,this) &&

	addSubscribedTopic<const geometry_msgs::PoseStamped::ConstPtr&, NeuralNetwork>(nodeHandler,poseTopic,
		&NeuralNetwork::receivedRobotPose,this)	&&

	addSubscribedTopic<const std_msgs::Bool::ConstPtr&, NeuralNetwork>(nodeHandler,updateWorldTopic,
		&NeuralNetwork::receivedUpdateWorld,this);
}

bool NeuralNetwork::createPublishers() {
	ROS_INFO("Creating publishers");
	return addPublisherClient<nav_msgs::OccupancyGrid>(
		nodeHandler, occupancyGridTopic, false) &&

	addPublisherClient<std_msgs::Float32MultiArray>(
		nodeHandler, neuralGridTopic, false);
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

void NeuralNetwork::receivedUpdateWorld(const std_msgs::Bool::ConstPtr& updateWorld) {
	this->updateWorld = updateWorld;
}

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

//	 const unsigned int num_input = 180;
//	 const unsigned int num_output = 8;
//	 const unsigned int num_layers = 3;
//	 const unsigned int num_neurons_hidden = 361;
//	 const float desired_error = (const float) 0.001;
//	 const unsigned int max_epochs = 500000;
//	 const unsigned int epochs_between_reports = 100;
//
//	 struct fann *ann = fann_create_standard(num_layers, num_input, num_neurons_hidden, num_output);
//
//	 fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
//	 fann_set_activation_function_output(ann, FANN_SIGMOID);
//
//	 fann_set_learning_rate(ann,0.1);
//	 fann_set_learning_momentum(ann,0.5);
//
//	 std::string neuralNetworkDataFile = get_current_dir_name();
//	 neuralNetworkDataFile =
//		neuralNetworkDataFile.erase(neuralNetworkDataFile.find("catkin_ws") + 9)
//	    .append("/src/neural_network/NewRNATrain.data");
//
//	 fann_train_on_file(ann, neuralNetworkDataFile.c_str(), max_epochs, epochs_between_reports, desired_error);
//
//	 std::string neuralNetowrkFile = get_current_dir_name();
//	 neuralNetowrkFile =
//		neuralNetowrkFile.erase(neuralNetowrkFile.find("catkin_ws") + 9)
//	    .append("/src/neural_network/rna.net");
//
//	 fann_save(ann, neuralNetowrkFile.c_str());
//
//	 fann_destroy(ann);
//
//	 return 0;

}
