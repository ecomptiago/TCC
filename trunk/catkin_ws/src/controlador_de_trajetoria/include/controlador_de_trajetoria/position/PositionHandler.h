/*
 * PositionHandler.h
 *
 *  Created on: Aug 25, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_POSITION_POSITIONHANDLER_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_POSITION_POSITIONHANDLER_H_
#define VREP_SIMULATION

#ifdef VREP_SIMULATION
	#include "geometry_msgs/PoseStamped.h"
#else
	#include "nav_msgs/Odometry.h"
#endif

#include "std_msgs/String.h"
#include "common/BaseRosNode.h"
#include "common/Position.h"

const char* actualRobotPositionTopic = "Position_handler/actual_robot_position";
const char* poseTopic = "/RosAria/pose";
const char* timerActualRobotPosition = "actualRobotPositionTimer";

class PositionHandler :public BaseRosNode{
	private:
		//Attributes
		/**TODO - Pass this attributed to base class
		to be inherited by derivated class. The actual
		problem is that it can not be instantied before
		calling ros::init*/
		ros::NodeHandle nodeHandler;
		common::Position position;
		float actualRobotPositionDelay; //This is in seconds

	public:
		//Constructors
		PositionHandler(int argc,char **argv, float actualRobotPositionDelay);

		//Destructor
		virtual ~PositionHandler() {} ;

		//Methods
		virtual int runNode();
		bool subscribeToTopics();
		bool createPublishers();
		bool createTimers();

		#ifdef VREP_SIMULATION
			void transformOdometryToPosition
				(const geometry_msgs::PoseStamped::ConstPtr& odometryPosition);
		#else
			void transformOdometryToPosition
				(const nav_msgs::Odometry::ConstPtr& odometryPosition);
		#endif

		void publishPosition(const ros::TimerEvent& timerEvent);

};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_POSITION_POSITIONHANDLER_H_ */
