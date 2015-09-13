#include <controlador_de_trajetoria/movimentation/MovimentationExecutor.h>

//Constructors
MovimentationExecutor::MovimentationExecutor(int argc, char **argv,
	float nextTryInterval, double velocity, double wakeUpTime,
	double verifyRobotMovimentDelay)
	: BaseRosNode(argc,argv,nodeName) {
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
				ROS_INFO("Moving robot to position");
				verifyMotorState();
				rotateRobot();
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
		publisherMap[targetPositionAchievedTopic].publish(positionAchieved);
}

void MovimentationExecutor::verifyMotorState() {
	if(motorEnabled == false) {
		std_srvs::Empty empty;
		servicesMap[enableMotorService].call(empty);
	}
}

void MovimentationExecutor::rotateRobot() {
	double angleTargetPosition =
		atan2(pointerTargetPosition->y,pointerTargetPosition->x) *
		180/M_PI;
	double actualAngle =
		tf::getYaw(actualOdometryPosition.pose.pose.orientation) *
		180/M_PI;
	ROS_INFO("Rotating robot to %f degrees",angleTargetPosition);
	publisherMap[cmdVelTopic].publish(createRotateMessage());
	while(!(actualAngle > angleTargetPosition - angleErrorMargin &&
		actualAngle < angleTargetPosition + angleErrorMargin)) {
			usleep(100000);
			ros::spinOnce();
			actualAngle =
				tf::getYaw(actualOdometryPosition.pose.pose.orientation) *
				180/M_PI;
	}
	publisherMap[cmdVelTopic].publish(createStopMessage());
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
	ROS_INFO("Moving robot to position x: %f y: %f from actual position",
			pointerTargetPosition->x,pointerTargetPosition->y);
	publisherMap[cmdVelTopic].publish(createMoveMessage(velocity));
	while(!(actualOdometryPosition.pose.pose.position.x > targetXAdjusted - positionErrorMargin &&
		actualOdometryPosition.pose.pose.position.x < targetXAdjusted + positionErrorMargin &&
		actualOdometryPosition.pose.pose.position.y > targetYAdjusted - positionErrorMargin &&
		actualOdometryPosition.pose.pose.position.y < targetYAdjusted + positionErrorMargin)) {
			usleep(100000);
			ros::spinOnce();
	}
	publisherMap[cmdVelTopic].publish(createStopMessage());
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
		10,&MovimentationExecutor::receivedActualOdometryRobotPosition,this);
	if(sub && sub2 && sub3) {
		subscriberMap[motorStateTopic] = sub;
		subscriberMap[targetPositionTopic] = sub2;
		subscriberMap[poseTopic] = sub3;
		return true;
	} else {
		ROS_DEBUG("Could not subscribe to all topics");
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
	if(pub && pub2 && pub3) {
		publisherMap[targetPositionAchievedTopic] =  pub;
		publisherMap[targetPositionTopic] =  pub2;
		publisherMap[cmdVelTopic] =  pub3;
		return true;
	} else {
		ROS_DEBUG("Could not create all publishers");
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
		servicesMap[enableMotorService] = service;
		servicesMap[disableMotorService] = service2;
		return true;
	} else {
		ROS_DEBUG("Could not create all services");
		return false;
	}
}

bool MovimentationExecutor::createTimers() {
	ROS_INFO("Creating timers");
	ros::Timer timer =
		nodeHandler.createTimer(ros::Duration(verifyRobotMovimentDelay),
		&PositionHandler::publishPosition,
		this,false);
	if(timer) {
		timerMap[actualRobotPositionTopic] = timer;
		return true;
	} else {
		ROS_DEBUG("Could not create all timers");
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
	if(pointerTargetPosition != NULL &&
		actualOdometryPosition.pose.pose.position.x == lastPosition.x
		&& actualOdometryPosition.pose.pose.position.y == lastPosition.y) {

	}
}

//Main
int main(int argc,char **argv) {
	try {
		MovimentationExecutor movimentationExecutor(argc,argv,1,1,0.1,0.1);
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
