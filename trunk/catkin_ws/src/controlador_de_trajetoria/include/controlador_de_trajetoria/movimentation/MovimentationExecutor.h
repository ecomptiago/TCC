/*
 * MovimentationExecutor.h
 *
 *  Created on: Aug 25, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_MOVIMENTATIONEXECUTOR_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_MOVIMENTATIONEXECUTOR_H_

#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "controlador_de_trajetoria/BaseRosNode.h"
#include "controlador_de_trajetoria/Position.h"
#include "controlador_de_trajetoria/Move_robot.h"

const char* nodeName = "Movimentation_executor";
const char* motorStateTopic = "/RosAria/motor_state";
const char* cmdVelTopic = "/RosAria/cmd_vel";
const char* poseTopic = "/RosAria/pose";
const char* actualRobotPositionTopic = "Position_handler/actual_robot_position";
const char* targetPositionAchievedTopic = "Message_handler/target_position_achieved";
const char* targetPositionTopic = "Message_handler/target_position";
//const char* movimentNotPossibleTopic = "Movimentation_executor/moviment_not_possible_cause";


class MovimentationExecutor :public BaseRosNode{
	private:
		//Attributes
		/**TODO - Pass this attributed to base class
		to be inherited by derivated class. The actual
		problem is that it can not be instantied before
		calling ros::init*/
		ros::NodeHandle nodeHandler;
		controlador_de_trajetoria::Position *pointerTargetPosition;
		nav_msgs::Odometry* pointerActualOdometryPosition;
		float nextTryInterval; // Time in seconds
		double velocity; // Velocity in m/s
		std_msgs::Bool motorState;
		double wakeUpTime;
		bool targetAchieved;
		double angle; //angle in degree

		//Methods
		void verifyMotorState();
		void rotateRobot();
		void moveRobot();

	public:
		//Constructors
		MovimentationExecutor(int argc,char **argv,
			float nextTryInterval, double velocity,
			double wakeUpTime);

		//Destructor
		virtual ~MovimentationExecutor() {} ;

		//Methods
		virtual int runNode();
		bool subscribeToTopics();
		bool createPublishers();
		void receivedActualOdometryRobotPosition(const
			nav_msgs::Odometry::ConstPtr& actualOdometryRobotPositionPointer);
		void receivedTargetPosition(const
			controlador_de_trajetoria::Move_robot::ConstPtr& targetPositionPointer);
		void receivedMotorState(const std_msgs::Bool::ConstPtr& motorStatePointer);
};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_MOVIMENTATIONEXECUTOR_H_ */
