#include <controlador_de_trajetoria/movimentation/MovimentationExecutor.h>

//Constructors
MovimentationExecutor::MovimentationExecutor(int argc, char **argv,
	float nextTryInterval, double wakeUpTime, double verifyRobotMovimentDelay) : BaseRosNode(argc,argv,nodeName) {
		this->pointerTargetPosition = NULL;
		this->nextTryInterval = nextTryInterval;
		this->wakeUpTime = wakeUpTime;
		this->targetAchieved = true;
		this->motorEnabled = false;
		this->verifyRobotMovimentDelay = verifyRobotMovimentDelay;
		this->pidController = PIDMovimentController(0.1,0.3,-0.15);
}

//Methods
int MovimentationExecutor::runNode() {
	ros::Rate rate(1/wakeUpTime);
	ROS_INFO("Running node");
	while(ros::ok()) {
		if(pointerTargetPosition != NULL &&
			targetAchieved == true) {
				targetAchieved = false;
				ROS_INFO("Verifying motor state");
				verifyMotorState();
				ROS_INFO("Moving robot to position");
				moveRobot();
		}
		sleepAndSpin(rate);
	}
	return 0;
}

geometry_msgs::Twist MovimentationExecutor::createStopMessage() {
	geometry_msgs::Twist stopMessage;
	stopMessage.angular.x = 0;
	stopMessage.angular.y = 0;
	stopMessage.angular.z = 0;
	stopMessage.linear.x = 0;
	stopMessage.linear.y = 0;
	stopMessage.linear.z = 0;
	return stopMessage;
}

void MovimentationExecutor::publishPositionAchieved(
	double initialXPosition, double initialYPosition) {
		controlador_de_trajetoria::Position positionAchieved;

		#ifdef VREP_SIMULATION
			positionAchieved.x =
				actualOdometryPosition.pose.position.x - initialXPosition;
			positionAchieved.y =
				actualOdometryPosition.pose.position.y - initialYPosition;
		#else
			positionAchieved.x =
				actualOdometryPosition.pose.pose.position.x - initialXPosition;
			positionAchieved.y =
				actualOdometryPosition.pose.pose.position.y - initialYPosition;
		#endif

		if(hasPublisher(targetPositionAchievedTopic)) {
			publisherMap[targetPositionAchievedTopic].publish(positionAchieved);
		}
}

void MovimentationExecutor::verifyMotorState() {
	if(motorEnabled == false) {
		std_srvs::Empty empty;
		serviceClientsMap[enableMotorService].call(empty);
	}
}

double MovimentationExecutor::getActualAngle(int sleepBeforeActualize) {
	if(sleepBeforeActualize) {
		sleepAndSpin(100);
	}

	#ifdef VREP_SIMULATION
		tf::Quaternion quaternion(
			0, 0, actualOdometryPosition.pose.orientation.z,
			actualOdometryPosition.pose.orientation.w);
		return OdometryUtils::getAngleFromQuaternation(quaternion.normalize());
	#else
		return OdometryUtils::getAngleFromQuaternation(
			actualOdometryPosition.pose.pose.orientation);
	#endif

}

void MovimentationExecutor::moveRobot() {

	#ifdef VREP_SIMULATION
		ROS_DEBUG("Moving robot to position x: %f y: %f from actual position "
			"x:%f y:%f and angle:%f",pointerTargetPosition->x,pointerTargetPosition->y,
			actualOdometryPosition.pose.position.x,
			actualOdometryPosition.pose.position.y,
			getActualAngle(false));
		double initialXPosition = actualOdometryPosition.pose.position.x;
		double initialYPosition = actualOdometryPosition.pose.position.y;
	#else
		ROS_DEBUG("Moving robot to position x: %f y: %f from actual position "
			"x:%f y:%f and angle:%f",pointerTargetPosition->x,pointerTargetPosition->y,
			actualOdometryPosition.pose.pose.position.x,
			actualOdometryPosition.pose.pose.position.y,
			getActualAngle(false));
		double initialXPosition = actualOdometryPosition.pose.pose.position.x;
		double initialYPosition = actualOdometryPosition.pose.pose.position.y;
	#endif

	pidController.setTargetPosition(*pointerTargetPosition);
	pidController.calculateRhoAlphaBeta(actualOdometryPosition);

	#ifdef VREP_SIMULATION
	while(MatrixUtils::mod<float>(pidController.calculateError()) > 0.1) {
		if(hasPublisher(cmdVelTopic)) {
			publisherMap[cmdVelTopic].publish(pidController.calculateVelocities());
		}
		pidController.calculateRhoAlphaBeta(actualOdometryPosition);
		sleepAndSpin(750);
		ROS_DEBUG("Actual position x:%f y:%f",actualOdometryPosition.pose.position.x,
			actualOdometryPosition.pose.position.y);
	}
	#else
		while(!(actualOdometryPosition.pose.pose.position.x > targetXAdjusted - positionErrorMargin &&
				actualOdometryPosition.pose.pose.position.x < targetXAdjusted + positionErrorMargin &&
				actualOdometryPosition.pose.pose.position.y > targetYAdjusted - positionErrorMargin &&
				actualOdometryPosition.pose.pose.position.y < targetYAdjusted + positionErrorMargin)) {
					sleepAndSpin(100);
					ROS_DEBUG("Actual position x:%f y:%f",actualOdometryPosition.pose.pose.position.x,
						actualOdometryPosition.pose.pose.position.y);
			}
	#endif

	if(hasPublisher(cmdVelTopic)) {
		publisherMap[cmdVelTopic].publish(createStopMessage());
	}
	ROS_INFO("Stopping of move robot");
	publishPositionAchieved(initialXPosition,initialYPosition);
	pointerTargetPosition = NULL;
	targetAchieved = true;
}

bool MovimentationExecutor::subscribeToTopics() {
	ROS_INFO("Subscribing to topics");
	return addSubscribedTopic<const std_msgs::Bool::ConstPtr&, MovimentationExecutor>(nodeHandler,motorStateTopic,
		   &MovimentationExecutor::receivedMotorState,this) &&

		   addSubscribedTopic<const controlador_de_trajetoria::Move_robot::ConstPtr&,
		   MovimentationExecutor>(nodeHandler,targetPositionTopic, &MovimentationExecutor::receivedTargetPosition,this) &&

		   #ifdef VREP_SIMULATION
			   addSubscribedTopic<const geometry_msgs::PoseStamped::ConstPtr&, MovimentationExecutor>(nodeHandler,poseTopic,
			   &MovimentationExecutor::receivedActualOdometryRobotPosition,this);
		   #else
	   	       addSubscribedTopic<const nav_msgs::Odometry::ConstPtr&, MovimentationExecutor>(nodeHandler,poseTopic,
			   &MovimentationExecutor::receivedActualOdometryRobotPosition,this);
		   #endif

}

bool MovimentationExecutor::createPublishers() {
	ROS_INFO("Creating publishers");
	return addPublisherClient<controlador_de_trajetoria::Position>(
		   nodeHandler, targetPositionAchievedTopic, false) &&

		   addPublisherClient<controlador_de_trajetoria::Move_robot>(
		   nodeHandler, targetPositionTopic, false) &&

		   addPublisherClient<geometry_msgs::Twist>(
		   nodeHandler, cmdVelTopic, false) &&

		   addPublisherClient<controlador_de_trajetoria::Movimentation_error>(
		   nodeHandler, movimentNotPossibleTopic, false);
}

bool MovimentationExecutor::createServices() {
	ROS_INFO("Creating services");
	return addServiceClient<std_srvs::Empty>(nodeHandler, enableMotorService)&&

		   addServiceClient<std_srvs::Empty>(nodeHandler, disableMotorService);
}

bool MovimentationExecutor::createTimers() {
	ROS_INFO("Creating timers");
	return addTimer<const ros::TimerEvent&, MovimentationExecutor>(nodeHandler, verifyRobotMovimentDelay,
		   verifyRobotMovimentTimer, &MovimentationExecutor::verifyRobotMovimentEvent, this, false);
}

//Callback
#ifdef VREP_SIMULATION
	void MovimentationExecutor::receivedActualOdometryRobotPosition(
			const geometry_msgs::PoseStamped::ConstPtr& actualOdometryRobotPositionPointer) {
		actualOdometryPosition = *actualOdometryRobotPositionPointer;
	}
#else
	void MovimentationExecutor::receivedActualOdometryRobotPosition(
			const nav_msgs::Odometry::ConstPtr& actualOdometryRobotPositionPointer) {
		actualOdometryPosition = *actualOdometryRobotPositionPointer;
	}
#endif

void MovimentationExecutor::receivedTargetPosition(
	const controlador_de_trajetoria::Move_robot::ConstPtr& targetPositionPointer) {
		ROS_DEBUG("Received target position x:%f y:%f ",
			targetPositionPointer->x,targetPositionPointer->y);
		if(pointerTargetPosition == NULL && targetAchieved == true) {
			targetPosition.x = targetPositionPointer->x;
			targetPosition.y = targetPositionPointer->y;
			pointerTargetPosition = &targetPosition;
		} else {
			ROS_INFO("Already moving the robot to other position");
		}
}

void MovimentationExecutor::receivedMotorState(
	const std_msgs::Bool::ConstPtr& motorStatePointer) {
		motorEnabled = motorStatePointer->data;
}

void MovimentationExecutor::verifyRobotMovimentEvent(const ros::TimerEvent& timerEvent) {
	if(pointerTargetPosition != NULL) {

		#ifdef VREP_SIMULATION
			if(actualOdometryPosition.pose.position.x == lastPosition.pose.position.x &&
			   actualOdometryPosition.pose.position.y == lastPosition.pose.position.y &&
			   actualOdometryPosition.pose.orientation.z == lastPosition.pose.orientation.z &&
			   actualOdometryPosition.pose.orientation.w == lastPosition.pose.orientation.w) {
					ROS_DEBUG("Robot not moving.Actual positions x:%f y:%f z:%f w:%f ."
						"Last positions x:%f y:%f z:%f w:%f",actualOdometryPosition.pose.position.x,
					actualOdometryPosition.pose.position.y,
					actualOdometryPosition.pose.orientation.z,
					actualOdometryPosition.pose.orientation.w,lastPosition.pose.position.x,
					lastPosition.pose.position.y,lastPosition.pose.orientation.z,
					lastPosition.pose.orientation.w);
		#else
			if(actualOdometryPosition.pose.pose.position.x == lastPosition.pose.pose.position.x &&
			   actualOdometryPosition.pose.pose.position.y == lastPosition.pose.pose.position.y &&
			   actualOdometryPosition.pose.pose.orientation.z == lastPosition.pose.pose.orientation.z &&
			   actualOdometryPosition.pose.pose.orientation.w == lastPosition.pose.pose.orientation.w) {
					ROS_DEBUG("Robot not moving.Actual positions x:%f y:%f z:%f w:%f ."
						"Last positions x:%f y:%f z:%f w:%f",actualOdometryPosition.pose.pose.position.x,
					actualOdometryPosition.pose.pose.position.y,
					actualOdometryPosition.pose.pose.orientation.z,
					actualOdometryPosition.pose.pose.orientation.w,lastPosition.pose.pose.position.x,
					lastPosition.pose.pose.position.y,lastPosition.pose.pose.orientation.z,
					lastPosition.pose.pose.orientation.w);
		#endif

					controlador_de_trajetoria::Movimentation_error movimentationError;
					movimentationError.coordinateToMove = targetPosition;
					MovimentationErrorEnum error;
					if(motorEnabled == false) {
						movimentationError.whyCantMove =
							error.getStringFromEnum(MovimentationErrorEnum::MOTOR_DISABLED);
						verifyMotorState();
					} else {
						movimentationError.whyCantMove =
							error.getStringFromEnum(MovimentationErrorEnum::UNKNOW);
					}
					if(hasPublisher(movimentNotPossibleTopic)) {
						publisherMap[movimentNotPossibleTopic].publish(movimentationError);
					}
					lastPosition = actualOdometryPosition;
		}
	}
}

//Main
int main(int argc,char **argv) {
	try {
		MovimentationExecutor movimentationExecutor(argc, argv, 1, 1, 5);
		if(movimentationExecutor.subscribeToTopics() &&
			movimentationExecutor.createPublishers() &&
			movimentationExecutor.createServices() &&
			movimentationExecutor.createTimers()) {
				return movimentationExecutor.runNode();
		} else {
			BaseRosNode::shutdownAndExit(nodeName);
		}
	} catch (std::exception &e) {
		BaseRosNode::shutdownAndExit(nodeName);
	}
}
