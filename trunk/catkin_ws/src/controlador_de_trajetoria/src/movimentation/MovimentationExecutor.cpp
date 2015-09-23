#include <controlador_de_trajetoria/movimentation/MovimentationExecutor.h>

//Constructors
MovimentationExecutor::MovimentationExecutor(int argc, char **argv,
	float nextTryInterval, double velocity, double wakeUpTime,
	double verifyRobotMovimentDelay) : BaseRosNode(argc,argv,nodeName) {
		this->nextTryInterval = nextTryInterval;
		this->velocity = velocity;
		this->wakeUpTime = wakeUpTime;
		this->targetAchieved = true;
		this->motorEnabled = false;
		this->verifyRobotMovimentDelay = verifyRobotMovimentDelay;
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
				ROS_INFO("Rotating robot");
				rotateRobot();
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

geometry_msgs::Twist MovimentationExecutor::createRotateMessage() {
	geometry_msgs::Twist rotateMessage;
	rotateMessage.angular.x = 0;
	rotateMessage.angular.y = 0;
	rotateMessage.angular.z = 0.1;
	rotateMessage.linear.x = 0;
	rotateMessage.linear.y = 0;
	rotateMessage.linear.z = 0;
	return rotateMessage;
}

geometry_msgs::Twist MovimentationExecutor::createMoveMessage(
	double velocity) {
		geometry_msgs::Twist moveMessage;
		moveMessage.angular.x = 0;
		moveMessage.angular.y = 0;
		moveMessage.angular.z = 0;
		moveMessage.linear.x = velocity;
		moveMessage.linear.y = 0;
		moveMessage.linear.z = 0;
		return moveMessage;
}

void MovimentationExecutor::publishPositionAchieved(
	double initialXPosition, double initialYPosition) {
		controlador_de_trajetoria::Position positionAchieved;
		positionAchieved.x =
			actualOdometryPosition.pose.pose.position.x - initialXPosition;
		positionAchieved.y =
			actualOdometryPosition.pose.pose.position.y - initialYPosition;
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

double MovimentationExecutor::actualizingAngle(double actualAngle) {
	sleepAndSpin(100);
	actualAngle = tf::getYaw(actualOdometryPosition.pose.pose.orientation)
			* 180/M_PI;
	ROS_DEBUG("Actual angle:%f", actualAngle);
	return actualAngle;
}

void MovimentationExecutor::rotateRobot() {
	double angleTargetPosition =
		atan2(pointerTargetPosition->y,pointerTargetPosition->x) *
		180/M_PI;
	double actualAngle =
		tf::getYaw(actualOdometryPosition.pose.pose.orientation) *
		180/M_PI;
	double minAngleWithErrorMargin = angleTargetPosition - angleErrorMargin;
	double maxAngleWithErrorMargin = angleTargetPosition + angleErrorMargin;
	ROS_DEBUG("target position x:%f y:%f . Actual angle: %f "
		"angle to achieve:%f",pointerTargetPosition->x,
		pointerTargetPosition->y,actualAngle,angleTargetPosition);
	if(hasPublisher(cmdVelTopic)) {
		publisherMap[cmdVelTopic].publish(createRotateMessage());
	}
	if(minAngleWithErrorMargin < -180){
		double adjustedAngleMin = 180 + (minAngleWithErrorMargin + 180);
		while(!(actualAngle > adjustedAngleMin || actualAngle < maxAngleWithErrorMargin)) {
			actualAngle = actualizingAngle(actualAngle);
		}
	} else if(maxAngleWithErrorMargin > 180) {
		double adjustedAngleMax = (maxAngleWithErrorMargin - 180) - 180;
		while(!(actualAngle > minAngleWithErrorMargin || actualAngle < adjustedAngleMax)) {
			actualAngle = actualizingAngle(actualAngle);
		}
	} else {
		while(!(actualAngle > minAngleWithErrorMargin && actualAngle < maxAngleWithErrorMargin)) {
			actualAngle = actualizingAngle(actualAngle);
		}
	}
	if(hasPublisher(cmdVelTopic)) {
		publisherMap[cmdVelTopic].publish(createStopMessage());
	}
	ROS_INFO("Stopping of rotate robot");
}

void MovimentationExecutor::moveRobot() {
	double initialXPosition = actualOdometryPosition.pose.pose.position.x;
	double initialYPosition = actualOdometryPosition.pose.pose.position.y;
	double targetXAdjusted;
	double targetYAdjusted;
	if(pointerTargetPosition->x == 0) {
		targetXAdjusted = initialXPosition;
	} else {
		targetXAdjusted = initialXPosition + pointerTargetPosition->x;
	}
	if(pointerTargetPosition->y == 0) {
		targetYAdjusted = initialYPosition;
	} else {
		targetYAdjusted = initialYPosition + pointerTargetPosition->y;
	}
	ROS_DEBUG("Moving robot to position x: %f y: %f from actual position "
		"x:%f y:%f and angle:%f",pointerTargetPosition->x,pointerTargetPosition->y,
		actualOdometryPosition.pose.pose.position.x,
		actualOdometryPosition.pose.pose.position.y,
		tf::getYaw(actualOdometryPosition.pose.pose.orientation) * 	180/M_PI);
	if(hasPublisher(cmdVelTopic)) {
		publisherMap[cmdVelTopic].publish(createMoveMessage(velocity));
	}
	while(!(actualOdometryPosition.pose.pose.position.x > targetXAdjusted - positionErrorMargin &&
		actualOdometryPosition.pose.pose.position.x < targetXAdjusted + positionErrorMargin &&
		actualOdometryPosition.pose.pose.position.y > targetYAdjusted - positionErrorMargin &&
		actualOdometryPosition.pose.pose.position.y < targetYAdjusted + positionErrorMargin)) {
			sleepAndSpin(100);
			ROS_DEBUG("Actual position x:%f y:%f",actualOdometryPosition.pose.pose.position.x,
				actualOdometryPosition.pose.pose.position.y);
	}
	if(hasPublisher(cmdVelTopic)) {
		publisherMap[cmdVelTopic].publish(createStopMessage());
	}
	ROS_INFO("Stopping of move robot");
	publishPositionAchieved(initialXPosition,initialYPosition);
	pointerTargetPosition.reset();
	targetAchieved = true;
}

bool MovimentationExecutor::subscribeToTopics() {
	ROS_INFO("Subscribing to topics");
	return addSubscribedTopic<const std_msgs::Bool::ConstPtr&, MovimentationExecutor>(nodeHandler,motorStateTopic,
		   &MovimentationExecutor::receivedMotorState,this) &&

		   addSubscribedTopic<const controlador_de_trajetoria::Move_robot::ConstPtr&,
		   MovimentationExecutor>(nodeHandler,targetPositionTopic, &MovimentationExecutor::receivedTargetPosition,this) &&

		   addSubscribedTopic<const nav_msgs::Odometry::ConstPtr&, MovimentationExecutor>(nodeHandler,poseTopic,
		   &MovimentationExecutor::receivedActualOdometryRobotPosition,this);
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
void MovimentationExecutor::receivedActualOdometryRobotPosition(
		const nav_msgs::Odometry::ConstPtr& actualOdometryRobotPositionPointer) {
	actualOdometryPosition = *actualOdometryRobotPositionPointer;
}

void MovimentationExecutor::receivedTargetPosition(
	const controlador_de_trajetoria::Move_robot::ConstPtr& targetPositionPointer) {
		ROS_DEBUG("Received target position x:%f y:%f vel:%f",
			targetPositionPointer->x,targetPositionPointer->y,
			targetPositionPointer->vel);
		if(pointerTargetPosition == NULL && targetAchieved == true) {
			targetPosition.x = targetPositionPointer->x;
			targetPosition.y = targetPositionPointer->y;
			if(targetPositionPointer-> vel !=0) {
				velocity = targetPositionPointer->vel;
			}
			*pointerTargetPosition = targetPosition;
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
		MovimentationExecutor movimentationExecutor(argc,argv,1,1,0.1,5);
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
