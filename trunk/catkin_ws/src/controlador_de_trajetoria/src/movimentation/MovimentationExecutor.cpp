#include <controlador_de_trajetoria/movimentation/MovimentationExecutor.h>

//Constructors
MovimentationExecutor::MovimentationExecutor(int argc, char **argv,
	float nextTryInterval, double velocity, double wakeUpTime)
	: BaseRosNode(argc,argv,nodeName) {
	this->pointerTargetPosition = NULL;
	this->pointerActualOdometryPosition = NULL;
	this->nextTryInterval = nextTryInterval;
	this->velocity = velocity;
	this->wakeUpTime = wakeUpTime;
	this->targetAchieved = false;
	this->angle = 0;

	ros::Rate oneHundredMilisencondsSleep(10);
	this->pointerTo100MilisencodsSleep = &oneHundredMilisencondsSleep;

	pointerToStopMessage = createStopMessage();
	pointerToRotateMessage = createRotateMessage();
}

//Methods
int MovimentationExecutor::runNode() {
	ros::Rate rate(1/wakeUpTime);
	ROS_INFO("Running node");
	while(ros::ok()) {
		if(pointerTargetPosition != NULL &&
				targetAchieved == true) {
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

const geometry_msgs::Twist* MovimentationExecutor::createStopMessage() {
	geometry_msgs::Twist* stopMessage;
	stopMessage->angular.x = 0;
	stopMessage->angular.y = 0;
	stopMessage->angular.z = 0;
	stopMessage->linear.x = 0;
	stopMessage->linear.y = 0;
	stopMessage->linear.z = 0;
	return stopMessage;
}

const geometry_msgs::Twist* MovimentationExecutor::createRotateMessage() {
	geometry_msgs::Twist* rotateMessage;
	rotateMessage->angular.x = 0;
	rotateMessage->angular.y = 0;
	rotateMessage->angular.z = 0;
	rotateMessage->linear.x = 0;
	rotateMessage->linear.y = 0;
	rotateMessage->linear.z = 0.1;
	return rotateMessage;
}

boost::shared_ptr<geometry_msgs::Twist> MovimentationExecutor::createMoveMessage(
		double velocity) {
	boost::shared_ptr<geometry_msgs::Twist> moveMessage;
	moveMessage->angular.x = velocity;
	moveMessage->angular.y = 0;
	moveMessage->angular.z = 0;
	moveMessage->linear.x = 0;
	moveMessage->linear.y = 0;
	moveMessage->linear.z = 0;
	return moveMessage;
}

void MovimentationExecutor::verifyMotorState() {
}

void MovimentationExecutor::rotateRobot() {
	ROS_DEBUG("Calculating angle to target position");
	geometry_msgs::Pose actualPosition =
		pointerActualOdometryPosition->pose.pose;
	double angleTargetPosition =
		atan2(actualPosition.position.y,actualPosition.position.x) *
		180/M_PI;
	ROS_INFO("Rotating robot");
	publisherMap[cmdVelTopic].publish(*pointerToRotateMessage);
	pointerTo100MilisencodsSleep->reset();
	while(actualPosition.orientation.z != angleTargetPosition) {
		ros::spinOnce();
		pointerTo100MilisencodsSleep->sleep();
		ROS_DEBUG("Angle of %f not achieved",angleTargetPosition);
	}
	publisherMap[cmdVelTopic].publish(*pointerToStopMessage);
	ROS_INFO("Stopping of rotate robot");
}

void MovimentationExecutor::moveRobot() {
	ROS_INFO("Moving robot");
	geometry_msgs::Pose actualPosition =
			pointerActualOdometryPosition->pose.pose;
	pointerTo100MilisencodsSleep->reset();
	publisherMap[cmdVelTopic].publish(createMoveMessage(velocity));
	while(actualPosition.position.x != pointerTargetPosition->x &&
		actualPosition.position.x != pointerTargetPosition->y) {
			ros::spinOnce();
			pointerTo100MilisencodsSleep->sleep();
			ROS_DEBUG("Position x: %f y: %f not achieved",
				pointerTargetPosition->x,pointerTargetPosition->y);
	}
	publisherMap[cmdVelTopic].publish(*pointerToStopMessage);
	ROS_INFO("Stopping of move robot");
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
		nodeHandler.subscribe(actualRobotPositionTopic,
		1000,&MovimentationExecutor::receivedActualOdometryRobotPosition,this);
	if(sub && sub2 && sub3) {
		subscriberMap[motorStateTopic] = sub;
		subscriberMap[targetPositionTopic] = sub;
		subscriberMap[actualRobotPositionTopic] = sub;
		return true;
	} else {
		ROS_DEBUG("Could not subscribe to all topics");
		return false;
	}

	return true;
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

//Callback
void MovimentationExecutor::receivedActualOdometryRobotPosition(
		const nav_msgs::Odometry::ConstPtr& actualOdometryRobotPositionPointer) {
	ROS_INFO("Received message of robot position");
	pointerActualOdometryPosition = const_cast<nav_msgs::Odometry*>(actualOdometryRobotPositionPointer.get());
	if(actualOdometryRobotPositionPointer->pose.pose.position.x
		== pointerTargetPosition->x &&
		actualOdometryRobotPositionPointer->pose.pose.position.y
		== pointerTargetPosition->y) {
		targetAchieved = true;
		pointerTargetPosition = NULL;
		ROS_INFO("target position achieved");
	} else {
		ROS_DEBUG("Robot did not arrived in the destination yet.");
	}
}

void MovimentationExecutor::receivedTargetPosition(
		const controlador_de_trajetoria::Move_robot::ConstPtr& targetPositionPointer) {
	ROS_INFO("Received next target position");
	targetAchieved = false;
	pointerTargetPosition->x = targetPositionPointer->x;
	pointerTargetPosition->y = targetPositionPointer->y;
	velocity = targetPositionPointer->vel;
}

void MovimentationExecutor::receivedMotorState(
		const std_msgs::Bool::ConstPtr& motorStatePointer) {
	ROS_INFO("Received motor state message");
	motorState = *motorStatePointer;
}


//Main
int main(int argc,char **argv) {
	try {
		MovimentationExecutor movimentationExecutor(argc,argv,1,1,1);

		if(movimentationExecutor.subscribeToTopics() &&
				movimentationExecutor.createPublishers()) {
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
