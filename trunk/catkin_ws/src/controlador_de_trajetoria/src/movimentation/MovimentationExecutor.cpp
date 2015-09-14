#include <controlador_de_trajetoria/movimentation/MovimentationExecutor.h>

//Constructors
MovimentationExecutor::MovimentationExecutor(int argc, char **argv,
	float nextTryInterval, double velocity, double wakeUpTime,
	double verifyRobotMovimentDelay) : BaseRosNode(argc,argv,nodeName) {
		this->pointerTargetPosition = NULL;
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
		ros::spinOnce();
		rate.sleep();
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
	usleep(100000);
	ros::spinOnce();
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
			usleep(100000);
			ros::spinOnce();
			ROS_DEBUG("Actual position x:%f y:%f",actualOdometryPosition.pose.pose.position.x,
				actualOdometryPosition.pose.pose.position.y);
	}
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
	ros::Subscriber sub =
		nodeHandler.subscribe(motorStateTopic,
		1000,&MovimentationExecutor::receivedMotorState,this);
	ros::Subscriber sub2 =
		nodeHandler.subscribe(targetPositionTopic,
		1000,&MovimentationExecutor::receivedTargetPosition,this);
	ros::Subscriber sub3 =
		nodeHandler.subscribe(poseTopic,
		1000,&MovimentationExecutor::receivedActualOdometryRobotPosition,this);
	if(sub && sub2 && sub3) {
		subscriberMap[motorStateTopic] = sub;
		subscriberMap[targetPositionTopic] = sub2;
		subscriberMap[poseTopic] = sub3;
		return true;
	} else {
		ROS_INFO("Could not subscribe to all topics");
		return false;
	}
}

bool MovimentationExecutor::createPublishers() {
	ROS_INFO("Creating publishers");
	ros::Publisher pub =
		nodeHandler.advertise<controlador_de_trajetoria::Position>(
		targetPositionAchievedTopic, 1000);
	ros::Publisher pub2 =
		nodeHandler.advertise<controlador_de_trajetoria::Move_robot>(
		targetPositionTopic, 1000);
	ros::Publisher pub3 =
		nodeHandler.advertise<geometry_msgs::Twist>(
		cmdVelTopic, 1000);
	ros::Publisher pub4 =
		nodeHandler.advertise<controlador_de_trajetoria::Movimentation_error>(
		movimentNotPossibleTopic, 1000);
	if(pub && pub2 && pub3 && pub4) {
		publisherMap[targetPositionAchievedTopic] =  pub;
		publisherMap[targetPositionTopic] =  pub2;
		publisherMap[cmdVelTopic] =  pub3;
		publisherMap[movimentNotPossibleTopic] =  pub4;
		return true;
	} else {
		ROS_INFO("Could not create all publishers");
		return false;
	}
}

bool MovimentationExecutor::createServices() {
	ROS_INFO("Creating services");
	ros::ServiceClient service =
		nodeHandler.serviceClient<std_srvs::Empty>(enableMotorService);
	ros::ServiceClient service2 =
		nodeHandler.serviceClient<std_srvs::Empty>(disableMotorService);
	if(service && service2) {
		serviceClientsMap[enableMotorService] = service;
		serviceClientsMap[disableMotorService] = service2;
		return true;
	} else {
		ROS_INFO("Could not create all services");
		return false;
	}
}

bool MovimentationExecutor::createTimers() {
	ROS_INFO("Creating timers");
	ros::Timer timer =
		nodeHandler.createTimer(ros::Duration(verifyRobotMovimentDelay),
		&MovimentationExecutor::verifyRobotMovimentEvent,
		this,false);
	if(timer) {
		timerMap[verifyRobotMovimentTimer] = timer;
		return true;
	} else {
		ROS_INFO("Could not create all timers");
		return false;
	}
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
			ros::shutdown();
			return 0;
		}
	} catch (std::exception &e) {
		ros::shutdown();
		return 0;
	}
}
