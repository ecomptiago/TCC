/*
 * PathDrawer.cpp
 *
 *  Created on: Jul 29, 2016
 *      Author: tcoelho
 */

#include "path_planner/PathDrawer.h"

//Constructors
PathDrawer::PathDrawer(int argc, char **argv, float robotPathDelay,float robotPositionDelay) :
	BaseRosNode(argc, argv, "Path_drawer"){
		this->robotPathDelay = robotPathDelay;
		this->robotPositionDelay = robotPositionDelay;
		path = nav_msgs::Path();
		path.header = std_msgs::Header();
		path.header.stamp = ros::Time::now();
		path.header.frame_id = "LaserScannerBody_2D";
}

//Methods
int PathDrawer::runNode() {
	ROS_INFO("Running node");

	if (VRepUtils::getObjectHandle(pionnerHandle,nodeHandler,signalObjectMap)) {
		ros::spin();
		return 0;
	} else {
		return shutdownAndExit();
	}

}


bool PathDrawer::subscribeToTopics() {
	return true;
}

bool PathDrawer::createServices() {
	return true;
}

bool PathDrawer::createServiceClients() {
	return true;
}

bool PathDrawer::createServiceServers() {
	return true;
}

bool PathDrawer::createPublishers() {
	return addPublisherClient<nav_msgs::Path>(nodeHandler,pathTopic,false);
}

bool PathDrawer::createTimers() {
	ROS_INFO("Creating timers");
	return addTimer<const ros::TimerEvent&, PathDrawer>(nodeHandler, robotPathDelay,
		   timerRobotPath, &PathDrawer::publishPath, this, false) &&
		   addTimer<const ros::TimerEvent&, PathDrawer>(nodeHandler, robotPositionDelay,
		   timerRobotPosition, &PathDrawer::getPosition, this, false);
}

void PathDrawer::publishPath(const ros::TimerEvent& timerEvent) {
	publisherMap[pathTopic].publish(path);
}

void PathDrawer::getPosition(const ros::TimerEvent& timerEvent) {
	common::simRosGetObjectPose simRosGetObjectPose;
	if(VRepUtils::getObjectPose(signalObjectMap[pionnerHandle], nodeHandler,simRosGetObjectPose)) {
//		simRosGetObjectPose.request.pose
		path.poses.push_back(simRosGetObjectPose.response.pose);
	}
}

//Callback

//Main
int main(int argc, char **argv) {
	PathDrawer pathDrawer(argc,argv,1,0.25);
	try{
		if(pathDrawer.createServices() &&
			pathDrawer.subscribeToTopics() &&
			pathDrawer.createPublishers() &&
			pathDrawer.createTimers()) {
				return pathDrawer.runNode();
		} else {
			return pathDrawer.shutdownAndExit();
		}
	} catch (std::exception &e) {
		return pathDrawer.shutdownAndExit(e);
	}
}



