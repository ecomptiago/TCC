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


void MovimentationExecutor::verifyMotorState() {
}

void MovimentationExecutor::rotateRobot() {
	ROS_DEBUG("Calculating what the angle to rotate");
	if(pointerTargetPosition->x ==
			pointerActualOdometryPosition->pose.pose.position.x) {
	} else if(pointerTargetPosition->y ==
			pointerActualOdometryPosition->pose.pose.position.y) {
	} else {
	}
}

void MovimentationExecutor::moveRobot() {
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
