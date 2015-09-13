/*
 * MovimentationExecutor.h
 *
 *  Created on: Aug 25, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_MOVIMENTATIONEXECUTOR_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_MOVIMENTATIONEXECUTOR_H_

#include "math.h"
#include "unistd.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "controlador_de_trajetoria/BaseRosNode.h"
#include "controlador_de_trajetoria/Position.h"
#include "controlador_de_trajetoria/Move_robot.h"
#include "controlador_de_trajetoria/Movimentation_error.h"
#include "controlador_de_trajetoria/movimentation/MovimentationErrorEnum.h"
#include "tf/transform_datatypes.h"

const char* nodeName = "Movimentation_executor";
const char* motorStateTopic = "/RosAria/motor_state";
const char* cmdVelTopic = "/RosAria/cmd_vel";
const char* poseTopic = "/RosAria/pose";
const char* enableMotorService = "/RosAria/enable_motors";
const char* disableMotorService = "/RosAria/disable_motors";
const char* targetPositionAchievedTopic = "Message_handler/target_position_achieved";
const char* targetPositionTopic = "Message_handler/target_position";
const char* movimentNotPossibleTopic = "Movimentation_executor/moviment_not_possible_cause";
const char* verifyRobotMovimentTimer = "verifyRobotMovimentTimer";

//TODO - use only shared_ptr instead of raw pointer (*)
class MovimentationExecutor :public BaseRosNode{
	private:
		//Attributes
		/**TODO - Pass this attributed to base class
		to be inherited by derivated class. The actual
		problem is that it can not be instantied before
		calling ros::init*/
		ros::NodeHandle nodeHandler;
		controlador_de_trajetoria::Position targetPosition;
		const controlador_de_trajetoria::Position* pointerTargetPosition;
		nav_msgs::Odometry actualOdometryPosition;
		float nextTryInterval; // Time in seconds
		double velocity; // Velocity in m/s
		bool motorEnabled;
		double wakeUpTime; // wakeUp in seconds
		bool targetAchieved;
		bool verifyRobotMovimentDelay; // this is in seconds
		nav_msgs::Odometry lastPosition;

		//Methods
		void verifyMotorState();
		void rotateRobot();
		void moveRobot();
		void publishPositionAchieved(
			double initialXPosition, double initialYPosition);
		geometry_msgs::Twist createStopMessage();
		geometry_msgs::Twist createRotateMessage();
		geometry_msgs::Twist createMoveMessage(double velocity);

	public:
		//Constructors
		MovimentationExecutor(int argc,char **argv,
			float nextTryInterval, double velocity,
			double wakeUpTime,double verifyRobotMovimentDelay);

		//Destructor
		//TODO - Delete all pointers to deallocate memory
		virtual ~MovimentationExecutor() {} ;

		//Methods
		virtual int runNode();
		bool subscribeToTopics();
		bool createPublishers();
		bool createServices();
		bool createTimers();
		void receivedActualOdometryRobotPosition(const
			nav_msgs::Odometry::ConstPtr& actualOdometryRobotPositionPointer);
		void receivedTargetPosition(const
			controlador_de_trajetoria::Move_robot::ConstPtr& targetPositionPointer);
		void receivedMotorState(const std_msgs::Bool::ConstPtr& motorStatePointer);
		void verifyRobotMovimentEvent(const ros::TimerEvent& timerEvent);
};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_MOVIMENTATIONEXECUTOR_H_ */
