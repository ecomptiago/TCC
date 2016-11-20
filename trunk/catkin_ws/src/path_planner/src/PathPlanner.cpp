/*
 * pathPlanner.cpp
 *
 *  Created on: Dec 15, 2015
 *      Author: tiago
 */

#include "path_planner/PathPlanner.h"

//Constructors
PathPlanner::PathPlanner(int argc, char **argv)
	: BaseRosNode(argc, argv, "Path_planner"){
		this->aStar = AStar();
}

//Methods
int PathPlanner::runNode() {
	ROS_INFO("Running node");

	ros::spin();

 	return shutdownAndExit();
}

bool PathPlanner::subscribeToTopics() {
	return addSubscribedTopic<const geometry_msgs::PoseStamped::ConstPtr&,PathPlanner>(nodeHandler,poseTopic,
		&PathPlanner::receivedRobotPose,this) &&

		addSubscribedTopic<const nav_msgs::OccupancyGrid::ConstPtr&,PathPlanner>(nodeHandler,occupancyGridTopic,
			&PathPlanner::receivedOccupancyGrid,this);
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
	return true;
}

bool PathPlanner::createServiceServers() {
	return addServiceServer<common::pathToTarget::Request&,
		common::pathToTarget::Response&, PathPlanner>(nodeHandler,bestPathService, &PathPlanner::bestPath,this);
}

bool PathPlanner::createPublishers() {
	return true;
}

//Callback
void PathPlanner::receivedRobotPose(const geometry_msgs::PoseStamped::ConstPtr& robotPose){
	this->robotPose.header = robotPose->header;
	this->robotPose.pose = robotPose->pose;
}

void PathPlanner::receivedOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& occupancyGrid) {
	this->occupancyGrid.info = occupancyGrid->info;
	this->occupancyGrid.header = occupancyGrid->header;
	this->occupancyGrid.data = occupancyGrid->data;
}

bool PathPlanner::bestPath(common::pathToTarget::Request  &req,
	common::pathToTarget::Response &res) {
		if (VRepUtils::getObjectHandle(pionnerHandle,nodeHandler,signalObjectMap)) {
			common::simRosGetObjectPose simRosGetObjectPose;
			aStar.setOccupancyGrid(occupancyGrid);
			if(VRepUtils::getObjectPose(signalObjectMap[pionnerHandle], nodeHandler,simRosGetObjectPose)) {
				common::Position targetPosition;
				targetPosition.x = req.x;
				targetPosition.y = req.y;

				common::Position initialPosition;
				initialPosition.x = simRosGetObjectPose.response.pose.pose.position.x;
				initialPosition.y = simRosGetObjectPose.response.pose.pose.position.y;

				if(aStar.findPathToGoal(initialPosition ,targetPosition)) {
					std::vector<AStarGridCell> path;
					aStar.reconstructPath(path, targetPosition,initialPosition);

					int charsWrote = 0;
					char buffer [occupancyGrid.data.size() * 6];
					res.path.resize(path.size());

					for(int i = 0; i < path.size();i++) {
						int cellPosition = path[i].cellGridPosition;
						res.path[i] = cellPosition;
						charsWrote += sprintf(buffer + charsWrote,
							" %d,",cellPosition);
					}
					ROS_DEBUG("Optimized path: [%s]",buffer);
					ROS_DEBUG("Found path to (%f,%f)",req.x,req.y);
				} else {
					int targetCellPosition =
							GridUtils::getDataVectorPosition(occupancyGrid, targetPosition);
					if(occupancyGrid.data[targetCellPosition] == unknownCell) {
						res.path.clear();
						ROS_INFO("Could not find path to (%f,%f)",req.x,req.y);
					} else {
						res.path.resize(1);
						res.path[0] = -2;
					}
				}
			} else {
				res.path[0] = -3;
				res.path.clear();
				ROS_INFO("Could not get robot position");
			}
		} else {
			res.path[0] = -3;
			res.path.clear();
			ROS_INFO("Could not get robot v-rep handler");
		}
		return true;
}

//Main
int main(int argc, char **argv) {
	PathPlanner pathPlanner(argc,argv);
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
