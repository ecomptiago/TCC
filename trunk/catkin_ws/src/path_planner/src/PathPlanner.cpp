/*
 * pathPlanner.cpp
 *
 *  Created on: Dec 15, 2015
 *      Author: tiago
 */

#include "path_planner/PathPlanner.h"

//Constructors
PathPlanner::PathPlanner(int argc, char **argv, int cellArea, int mapWidth, int mapHeight,
	float angleTolerance, double wakeUpTime) : BaseRosNode(argc, argv, "Path_planner"){
		this->occupancyGrid.info.resolution = cellArea;
		this->occupancyGrid.info.width = mapWidth;
		this->occupancyGrid.info.height = mapHeight;
		this->occupancyGrid.header.frame_id = "LaserScannerBody_2D";
		for(int i = 0; i < (mapWidth * mapHeight) / cellArea; i++) {
			this->occupancyGrid.data.insert(this->occupancyGrid.data.begin(),unknownCell);
		}
		this->angleTolerance = angleTolerance;
		this->wakeUpTime = wakeUpTime;
		this->aStar = AStar();
		this->neuralGrid.data.resize(49);
}

//Methods
int PathPlanner::runNode() {
	ROS_INFO("Running node");

	path_planner::simRosGetObjectChild simRosGetObjectChild;
	bool firstLoop = true;
	int index = 0;


	//Getting origin coordinates
//	if(VRepUtils::getObjectHandle(floorHandle,nodeHandler,signalObjectMap)) {
//		geometry_msgs::PoseStamped origin;
//		do{
//			simRosGetObjectChild.request.handle = signalObjectMap[floorHandle];
//			simRosGetObjectChild.request.index = index;
//			serviceClientsMap[getObjectChildService].call(simRosGetObjectChild);
//			int32_t childHandle = simRosGetObjectChild.response.childHandle;
//			if (childHandle != responseError) {
//				common::simRosGetObjectPose simRosGetObjectPose;
//				if(VRepUtils::getObjectPose(simRosGetObjectChild.response.childHandle,
//					nodeHandler,simRosGetObjectPose)) {
//						if(firstLoop || (
//							simRosGetObjectPose.response.pose.pose.position.x < origin.pose.position.x &&
//							simRosGetObjectPose.response.pose.pose.position.y < origin.pose.position.y)) {
//								origin = simRosGetObjectPose.response.pose;
//								firstLoop = false;
//						}
//				} else {
//					ROS_ERROR("Could not get pose of a children from object %s",floorHandle);
//					return shutdownAndExit();
//				}
//			}
//			index++;
//		} while(simRosGetObjectChild.response.childHandle != responseError);
//		occupancyGrid.info.origin.orientation = origin.pose.orientation;
//		occupancyGrid.info.origin.position.x = origin.pose.position.x - 2.5;
//		occupancyGrid.info.origin.position.y = origin.pose.position.y - 2.5;
//	} else {
//		ROS_ERROR("Could not found handle for object %s",floorHandle);
//		return shutdownAndExit();
//	}

/*
	if(VRepUtils::getObjectHandle(cuboidHandle,nodeHandler,signalObjectMap)) {
		index = 0;
		addObjectToOccupancyMap(signalObjectMap[cuboidHandle]);
		simRosGetObjectChild.request.handle = signalObjectMap[cuboidHandle];
		simRosGetObjectChild.request.index = index;
		serviceClientsMap[getObjectChildService].call(simRosGetObjectChild);
		do{
			int32_t childHandle = simRosGetObjectChild.response.childHandle;
			if(addObjectToOccupancyMap(childHandle)) {
				index++;
			} else {
				return shutdownAndExit();

			}
			simRosGetObjectChild.request.handle = signalObjectMap[cuboidHandle];
			simRosGetObjectChild.request.index = index;
			serviceClientsMap[getObjectChildService].call(simRosGetObjectChild);
		} while(simRosGetObjectChild.response.childHandle != responseError);
	} else {
		ROS_ERROR("Could not found handle for object %s",cuboidHandle);
		return shutdownAndExit();
	}

*/
	/*std::vector<int8_t>::iterator it;
	it = occupancyGrid.data.begin();
	char buffer [occupancyGrid.data.size() * 6];
	int charsWrote = 0;

	while(it != occupancyGrid.data.end()) {
		for(int i = 0;
			i < ceil(occupancyGrid.info.width / occupancyGrid.info.resolution);
			i++) {
				charsWrote += sprintf(buffer + charsWrote, "| %d |",*it);
				it++;
		}
		charsWrote += sprintf(buffer + charsWrote, "\n");
	}

	ROS_DEBUG("Map of static objects: \n%s",buffer);
*/
/*
	if (VRepUtils::getObjectHandle(pionnerHandle,nodeHandler,signalObjectMap)) {
		common::simRosGetObjectPose simRosGetObjectPose;
		aStar.setOccupancyGrid(occupancyGrid);
		if(VRepUtils::getObjectPose(signalObjectMap[pionnerHandle], nodeHandler,simRosGetObjectPose)) {
			common::Position targetPosition;
			targetPosition.x = 4.35;
			targetPosition.y = 10.14;

			common::Position initialPosition;
			initialPosition.x = simRosGetObjectPose.response.pose.pose.position.x;
			initialPosition.y = simRosGetObjectPose.response.pose.pose.position.y;

			if(aStar.findPathToGoal(initialPosition ,targetPosition)) {
				std::vector<AStarGridCell> path;
				aStar.reconstructPath(path, targetPosition,initialPosition);

				std::vector<AStarGridCell>::iterator it;
				it = path.begin();
				charsWrote = 0;

				while(it != path.end()) {
					charsWrote += sprintf(buffer + charsWrote,
						" %d,",((AStarGridCell)*it).cellGridPosition);
					geometry_msgs::PoseStamped poseStamped;
					it++;
				}
				ROS_DEBUG("Optimized path: [%s]",buffer);
			}
		}
	}

*/
	ros::Rate rate(1/wakeUpTime);

	occupancyGrid.info.resolution = 1;
	occupancyGrid.info.width = 10;
	occupancyGrid.info.height = 10;
	occupancyGrid.info.origin.position.x = -6.15000009537;
	occupancyGrid.info.origin.position.y = 0.724999427795;
	occupancyGrid.info.origin.orientation.x = 0;
	occupancyGrid.info.origin.orientation.y = 0;
	occupancyGrid.info.origin.orientation.z = 0;
	occupancyGrid.info.origin.orientation.w = 1;

//	tf::Quaternion quaternion = tf::createQuaternionFromYaw(32.5 * (M_PI / 180));
//	occupancyGrid.info.origin.orientation.x = quaternion.getX();
//	occupancyGrid.info.origin.orientation.y = quaternion.getY();
//	occupancyGrid.info.origin.orientation.z = quaternion.getZ();
//	occupancyGrid.info.origin.orientation.w = quaternion.getW();

//	for(int i = 0 ; i < 100; i++) {
//		occupancyGrid.data[i] = unknownCell;
//	}

 	while(ros::ok()) {
		sleepAndSpin(rate);
		publisherMap[mapTopic].publish(occupancyGrid);
 		float angle = OdometryUtils::getAngleFromQuaternation(
 	 		tf::Quaternion(0,0,
 			robotPose.pose.orientation.z,
 			robotPose.pose.orientation.w),false);
 		if(NumericUtils::isFirstLess<float>(angle,0.0)) {
 			angle = angle + 360;
 		}
 		ROS_DEBUG("Angle %f",angle);
 		angle = (angle * M_PI) / 180;

 		int i = 0;

 		for(float u = 1; u < 8; u++ ) {
 			for(float v = 3; v > -4; v--) {
 				float x = (u * cos(angle)) - (v * sin(angle));
 				float y = (u * sin(angle)) + (v * cos(angle));
 				x = x + robotPose.pose.position.x;
 				y = y + robotPose.pose.position.y;
// 				if(NumericUtils::isFirstLess<float>())
 				float cellValue;
 				common::Position position;
 				position.x = x;
 				position.y = y;
 				int cellPosition =
 					PathPlannerUtils::getDataVectorPosition(occupancyGrid,position);
 				ROS_DEBUG("Position.x %f , Position.y %f , cellPositio %d", x, y, cellPosition);
 				if(NumericUtils::isFirstLess<float>(neuralGrid.data[i],0.0)) {
 					cellValue = neuralGrid.data[i] * -1;
 				} else {
 					cellValue = neuralGrid.data[i];
 				}
 				if(NumericUtils::isFirstLessEqual<float>(cellValue, 0.5)) {
 					occupancyGrid.data[cellPosition] = freeCell;
 				} else {
 					occupancyGrid.data[cellPosition] = occupiedCell;
 				}
 				i++;
 				publisherMap[mapTopic].publish(occupancyGrid);
 			}
 		}
//		publisherMap[mapTopic].publish(occupancyGrid);
	}

	return shutdownAndExit();
}

bool PathPlanner::addObjectToOccupancyMap(int32_t childHandle) {
	if (childHandle != responseError) {
		common::simRosGetObjectPose simRosGetObjectPose;
		path_planner::ObjectInfo objectInfo;
		if (VRepUtils::getObjectPose(childHandle, nodeHandler,simRosGetObjectPose)) {
			tf::Quaternion quaternion(0, 0,
				simRosGetObjectPose.response.pose.pose.orientation.z,
				simRosGetObjectPose.response.pose.pose.orientation.w);
			if (getObjectWidthHeight(childHandle, objectInfo)) {
				common::Position objectPosition;
				if (getMinimumXYObjectCoordinate(childHandle,simRosGetObjectPose, objectPosition)) {
					float angle = OdometryUtils::getAngleFromQuaternation(quaternion, false);
					float cellSize = occupancyGrid.info.resolution;
					int positionToInsert = PathPlannerUtils::getDataVectorPosition(occupancyGrid, objectPosition);
					int mapWidth = round(occupancyGrid.info.width / cellSize);
					if ((NumericUtils::isFirstGreaterEqual<float>(angle, 0) &&
						NumericUtils::isFirstLessEqual<float>(angle, angleTolerance)) ||
						(NumericUtils::isFirstGreaterEqual<float>(angle, -angleTolerance) &&
						NumericUtils::isFirstLessEqual<float>(angle, 0)) ||
						(NumericUtils::isFirstGreaterEqual<float>(angle, 180 - angleTolerance) &&
						NumericUtils::isFirstLessEqual<float>(angle, 180)) ||
						(NumericUtils::isFirstGreaterEqual<float>(angle, angleTolerance - 180) &&
						NumericUtils::isFirstLessEqual<float>(angle, -180))) {
							int numberOfCellsOccupiedX = round(objectInfo.width / cellSize);
							int numberOfCellsOccupiedY;
							if (objectInfo.height < cellSize) {
								numberOfCellsOccupiedY = 1;
							} else {
								numberOfCellsOccupiedY = ceil(objectInfo.height / cellSize);
							}
							for(int i = 0; i < numberOfCellsOccupiedX; i++) {
								for(int j = 0; j <numberOfCellsOccupiedY; j++) {
									occupancyGrid.data.at(positionToInsert + i + (j * mapWidth)) = occupiedCell;
								}
							}
							return true;
					} else if ((NumericUtils::isFirstGreaterEqual<float>(angle, 90 - angleTolerance) &&
						NumericUtils::isFirstLessEqual<float>(angle, 90 + angleTolerance)) ||
						(NumericUtils::isFirstGreaterEqual<float>(angle, -90 - angleTolerance) &&
						NumericUtils::isFirstLessEqual<float>(angle, -90 + angleTolerance))) {
							//TODO
							return true;
					} else {
						//TODO
						return true;
					}
				} else {
					ROS_ERROR("Could not get coordinates of a children from object %s",
						cuboidHandle);
					return false;
				}
			} else {
				ROS_ERROR("Could not get pose of a children from object %s",
					cuboidHandle);
				return false;
			}
		} else {
			ROS_ERROR("Could not get pose of a children from object %s",
				cuboidHandle);
			return false;
		}
	} else {
		ROS_ERROR("Object handle can not be -1");
		return false;
	}
}

bool PathPlanner::getMinimumXYObjectCoordinate(int32_t objecthandle,
	common::simRosGetObjectPose &simRosGetObjectPose,common::Position &position){
	path_planner::ObjectInfo objectInfo;
	if(getObjectWidthHeight(objecthandle,objectInfo)) {
		if(objectInfo.height / 2 < 1) {
			position.x = simRosGetObjectPose.response.pose.pose.position.x - (objectInfo.width / 2);
			position.y = simRosGetObjectPose.response.pose.pose.position.y;
		} else if(objectInfo.width / 2 < 1) {
			position.x = simRosGetObjectPose.response.pose.pose.position.x;
			position.y = simRosGetObjectPose.response.pose.pose.position.y - (objectInfo.height / 2);
		} else {
			position.x = simRosGetObjectPose.response.pose.pose.position.x - (objectInfo.width / 2);
			position.y = simRosGetObjectPose.response.pose.pose.position.y - (objectInfo.height / 2);
		}
		return true;
	} else {
		return false;
	}
}

bool PathPlanner::getObjectWidthHeight(int32_t objectHandle,
	path_planner::ObjectInfo &objectInfo) {
		path_planner::simRosGetObjectFloatParameter simRosGetObjectFloatParameter;
		callGetFloatParameterService(objectHandle,sim_objfloatparam_modelbbox_min_x,simRosGetObjectFloatParameter);
		if(simRosGetObjectFloatParameter.response.result == responseError) {
			return false;
		} else {
			float minX = simRosGetObjectFloatParameter.response.parameterValue;
			callGetFloatParameterService(objectHandle,sim_objfloatparam_modelbbox_max_x,simRosGetObjectFloatParameter);
			if(simRosGetObjectFloatParameter.response.result == responseError) {
				return false;
			} else {
				objectInfo.width = simRosGetObjectFloatParameter.response.parameterValue - minX;
			}
		}
		callGetFloatParameterService(objectHandle,sim_objfloatparam_modelbbox_min_y,simRosGetObjectFloatParameter);
		if(simRosGetObjectFloatParameter.response.result == responseError) {
			return false;
		} else {
			float minY = simRosGetObjectFloatParameter.response.parameterValue;
			callGetFloatParameterService(objectHandle,sim_objfloatparam_modelbbox_max_y,simRosGetObjectFloatParameter);
			if(simRosGetObjectFloatParameter.response.result == responseError) {
				return false;
			} else {
				objectInfo.height = simRosGetObjectFloatParameter.response.parameterValue - minY;
				return true;
			}
		}
}

void PathPlanner::callGetFloatParameterService(int32_t objectHandle,
	int32_t parameterID,
	path_planner::simRosGetObjectFloatParameter &simRosGetObjectFloatParameter) {
		simRosGetObjectFloatParameter.request.handle = objectHandle;
		simRosGetObjectFloatParameter.request.parameterID =
				parameterID;
		serviceClientsMap[getObjectFloatParameterService].call(
			simRosGetObjectFloatParameter);
}

bool PathPlanner::subscribeToTopics() {
	return addSubscribedTopic<const geometry_msgs::PoseStamped::ConstPtr&,PathPlanner>(nodeHandler,poseTopic,
		&PathPlanner::receivedRobotPose,this) &&

		addSubscribedTopic<const std_msgs::Float32MultiArray::ConstPtr&,PathPlanner>(nodeHandler,neuralGridTopic,
			&PathPlanner::receivedNeuralGrid,this);
}

bool PathPlanner::createServices() {
	ROS_INFO("Creating services");
	if(createServiceClients()) {
		return createServiceServers();
	} else {
		return false;
	}
}

bool PathPlanner::createServiceClients() {
	return addServiceClient<path_planner::simRosGetObjectGroupData>(nodeHandler, getObjectGroupDataService) &&
		   addServiceClient<path_planner::simRosGetObjectFloatParameter>(nodeHandler, getObjectFloatParameterService) &&
		   addServiceClient<path_planner::simRosGetObjectChild>(nodeHandler,getObjectChildService);
}

bool PathPlanner::createServiceServers() {
	return true;
}

bool PathPlanner::createPublishers() {
	return addPublisherClient<nav_msgs::OccupancyGrid>(nodeHandler,mapTopic,false);
}

//Callback
void PathPlanner::receivedRobotPose(const geometry_msgs::PoseStamped::ConstPtr& robotPose){
	this->robotPose.header = robotPose->header;
	this->robotPose.pose = robotPose->pose;
}

void PathPlanner::receivedNeuralGrid(const std_msgs::Float32MultiArray::ConstPtr& neuralGrid) {
	for(int i = 0 ; i < neuralGrid->data.capacity(); i++) {
		this->neuralGrid.data[i] = neuralGrid->data[i];
	}
}


//Main
int main(int argc, char **argv) {
	PathPlanner pathPlanner(argc,argv,1,10,10,10,1);
	try{
		if(pathPlanner.createServices() &&
			pathPlanner.subscribeToTopics() &&
			pathPlanner.createPublishers()) {
				return pathPlanner.runNode();
		} else {
			return pathPlanner.shutdownAndExit();
		}
	} catch (std::exception &e) {
		return pathPlanner.shutdownAndExit(e);
	}
}
