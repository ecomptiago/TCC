/*
 * MovimentationExecutor.h
 *
 *  Created on: Aug 25, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_MOVIMENTATIONEXECUTOR_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_MOVIMENTATIONEXECUTOR_H_

#define VREP_SIMULATION

#include "math.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"

#ifdef VREP_SIMULATION
	#include "geometry_msgs/PoseStamped.h"
#else
	#include "nav_msgs/Odometry.h"
#endif

#include "common/BaseRosNode.h"
#include "common/utils/OdometryUtils.h"
#include "controlador_de_trajetoria/Position.h"
#include "controlador_de_trajetoria/Move_robot.h"
#include "controlador_de_trajetoria/Movimentation_error.h"
#include "controlador_de_trajetoria/movimentation/MovimentationErrorEnum.h"
#include "controller/MovimentControllerInterface.h"
#include "controller/PIDMovimentController.h"
#include "tf/transform_datatypes.h"

const char* nodeName = "Movimentation_executor";
const char* motorStateTopic = "/RosAria/motors_state";
const char* cmdVelTopic = "/RosAria/cmd_vel";
const char* poseTopic = "/RosAria/pose";
const char* enableMotorService = "/RosAria/enable_motors";
const char* disableMotorService = "/RosAria/disable_motors";
const char* targetPositionAchievedTopic = "Message_handler/target_position_achieved";
const char* targetPositionTopic = "Message_handler/target_position";
const char* movimentNotPossibleTopic = "Movimentation_executor/moviment_not_possible_cause";
const char* verifyRobotMovimentTimer = "verifyRobotMovimentTimer";

class MovimentationExecutor :public BaseRosNode{
	private:
		//Attributes

		/**TODO - Pass this attributed to base class
		to be inherited by derivated class. The actual
		problem is that it can not be instantied before
		calling ros::init*/
		ros::NodeHandle nodeHandler;
		controlador_de_trajetoria::Position targetPosition;

		//TODO - Use shared_ptr instead of raw pointer
		controlador_de_trajetoria::Position *pointerTargetPosition;

		PIDMovimentController pidController;

		#ifdef VREP_SIMULATION
			geometry_msgs::PoseStamped actualOdometryPosition;
			geometry_msgs::PoseStamped lastPosition;
		#else
			nav_msgs::Odometry actualOdometryPosition;
			nav_msgs::Odometry lastPosition;
		#endif

		float nextTryInterval; // Time in seconds
		bool motorEnabled;
		double wakeUpTime; // wakeUp in seconds
		bool targetAchieved;
		bool verifyRobotMovimentDelay; // this is in seconds

		//Methods
		void verifyMotorState();
		void rotateRobot();
		void moveRobot();
		void publishPositionAchieved(
			double initialXPosition, double initialYPosition);
		geometry_msgs::Twist createStopMessage();
		double getActualAngle(int sleepBeforeActualize);

	public:
		//Constructors
		MovimentationExecutor(int argc,char **argv,
			float nextTryInterval, double wakeUpTime,
			double verifyRobotMovimentDelay);

		//Destructor
		virtual ~MovimentationExecutor() {};

		//Methods
		virtual int runNode();
		bool subscribeToTopics();
		bool createPublishers();
		bool createServices();
		bool createTimers();

		#ifdef VREP_SIMULATION
			void receivedActualOdometryRobotPosition(
					const geometry_msgs::PoseStamped::ConstPtr& actualOdometryRobotPositionPointer);
		#else
			void receivedActualOdometryRobotPosition(
					const nav_msgs::Odometry::ConstPtr& actualOdometryRobotPositionPointer);
		#endif

		void receivedTargetPosition(const
			controlador_de_trajetoria::Move_robot::ConstPtr& targetPositionPointer);
		void receivedMotorState(const std_msgs::Bool::ConstPtr& motorStatePointer);
		void verifyRobotMovimentEvent(const ros::TimerEvent& timerEvent);
};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_MOVIMENTATIONEXECUTOR_H_ */
