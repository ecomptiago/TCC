#include "string"
#include "stdexcept"
#include "ros/ros.h"
#include "controlador_de_trajetoria/error/MethodNotImplementedError.h"
#include "controlador_de_trajetoria/position/PositionHandler.h"

//Constructors
PositionHandler::PositionHandler(int argc, char **argv) :
	BaseRosNode(argc,argv,getNodeName()) {
}


//Getters and setters
const std::string PositionHandler::getNodeName() {
	return "Position_handler";
}

//Methods
int PositionHandler::runNode() {
	return 0;
}

//Main
int main(int argc,char **argv) {
	try {
		PositionHandler positionHandler(argc,argv);
		return positionHandler.runNode();
	} catch (std::exception &e) {
		ros::shutdown();
		return 0;
	}
}
