/*
 * PositionHandler.h
 *
 *  Created on: Aug 25, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_POSITION_POSITIONHANDLER_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_POSITION_POSITIONHANDLER_H_

#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "controlador_de_trajetoria/BaseRosNode.h"
#include "controlador_de_trajetoria/Position.h"

const char* actualRobotPositionTopic = "Position_handler/actual_robot_position";
const char* poseTopic = "/RosAria/pose";
const char* nodeName = "Position_handler";
const char* timerActualRobotPosition = "actualRobotPositionTimer";
const float actualRobotPositionDelay = 1; // this is in seconds

class PositionHandler :public BaseRosNode{
	private:
		//Attributes
		/**TODO - Pass this attributed to base class
		to be inherited by derivated class. The actual
		problem is that it can not be instantied before
		calling ros::init*/
		ros::NodeHandle nodeHandler;
		controlador_de_trajetoria::Position position;

	public:
		//Constructors
		PositionHandler(int argc,char **argv);

		//Destructor
		virtual ~PositionHandler() {} ;

		//Methods
		virtual int runNode();
		bool subscribeToTopics();
		bool createPublishers();
		bool createTimers();
		void transformOdometryToPosition
			(const nav_msgs::Odometry::ConstPtr& odometryPosition);
		void publishPosition(const ros::TimerEvent& timerEvent);

};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_POSITION_POSITIONHANDLER_H_ */
