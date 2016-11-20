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
#include "functional"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"

#ifdef VREP_SIMULATION
	#include "geometry_msgs/PoseStamped.h"
#else
	#include "nav_msgs/Odometry.h"
#endif

#include "common/utils/MatrixUtils.h"
#include "common/BaseRosNode.h"
#include "common/utils/OdometryUtils.h"
#include "common/Position.h"
#include "common/Position.h"
#include "controller/MovimentControllerInterface.h"
#include "controller/ProportionalMovimentController.h"
#include "tf/transform_datatypes.h"

const char* poseTopic = "/RosAria/pose";
const char* targetPositionProportionalControllerTopic = "/MovimentationExecutor/target";
const char*	velTopic = "/MovimentationExecutor/velocity";
const char*	errorTopic = "/MovimentationExecutor/error";

class MovimentationExecutor :public BaseRosNode{
	private:
		//Attributes

		/**TODO - Pass this attributed to base class
		to be inherited by derivated class. The actual
		problem is that it can not be instantied before
		calling ros::init*/
		ros::NodeHandle nodeHandler;
		common::Position targetPosition;

		//TODO - Use shared_ptr instead of raw pointer
		common::Position *pointerTargetPosition;

		ProportionalMovimentController proportionalController;

		#ifdef VREP_SIMULATION
			geometry_msgs::PoseStamped actualOdometryPosition;
			geometry_msgs::PoseStamped lastPosition;
		#else
			nav_msgs::Odometry actualOdometryPosition;
			nav_msgs::Odometry lastPosition;
		#endif

		double wakeUpTime; // wakeUp in seconds

		//Methods
		double getActualAngle(bool sleepBeforeActualize);
		int getQuadrant(double angle);

	public:
		//Constructors
		MovimentationExecutor(int argc,char **argv,
			double wakeUpTime);

		//Destructor
		virtual ~MovimentationExecutor() {};

		//Methods
		virtual int runNode();
		bool subscribeToTopics();
		bool createPublishers();

		#ifdef VREP_SIMULATION
			void receivedActualOdometryRobotPosition(
				const geometry_msgs::PoseStamped::ConstPtr& actualOdometryRobotPositionPointer);
		#else
			void receivedActualOdometryRobotPosition(
					const nav_msgs::Odometry::ConstPtr& actualOdometryRobotPositionPointer);
		#endif

		void receivedTargetPosition(const
			common::Position::ConstPtr& targetPositionPointer);
};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_MOVIMENTATION_MOVIMENTATIONEXECUTOR_H_ */
