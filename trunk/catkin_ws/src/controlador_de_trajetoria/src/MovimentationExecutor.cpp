#include <controlador_de_trajetoria/MovimentationExecutor.h>

//Constructors
MovimentationExecutor::MovimentationExecutor(int argc, char **argv,
	double wakeUpTime) :
	BaseRosNode(argc,argv,"Movimentation_executor") {
		this->pointerTargetPosition = NULL;
		this->wakeUpTime = wakeUpTime;
		this->proportionalController = ProportionalMovimentController(0.2,0.6,-0.05);
}

//Methods
int MovimentationExecutor::runNode() {
	ros::Rate rate(1/wakeUpTime);
	ROS_INFO("Running node");
	while(ros::ok()) {
		if(pointerTargetPosition != NULL) {
			ROS_DEBUG("Moving robot to position x: %f y: %f from actual position "
				"x:%f y:%f and angle:%f",pointerTargetPosition->x,pointerTargetPosition->y,
				actualOdometryPosition.pose.position.x,
				actualOdometryPosition.pose.position.y,
				getActualAngle(false));
			proportionalController.setTargetPosition(*pointerTargetPosition);
			proportionalController.calculateRhoAlphaBeta(actualOdometryPosition);
			publisherMap[velTopic].publish(proportionalController.calculateVelocities());
			std_msgs::Float32 float32;
			float32.data = proportionalController.calculateError();
			publisherMap[errorTopic].publish(float32);
		}
		sleepAndSpin(rate);
	}
	return shutdownAndExit();
}


double MovimentationExecutor::getActualAngle(int sleepBeforeActualize) {
	if(sleepBeforeActualize) {
		sleepAndSpin(100);
	}

	#ifdef VREP_SIMULATION
		tf::Quaternion quaternion(
			0, 0, actualOdometryPosition.pose.orientation.z,
			actualOdometryPosition.pose.orientation.w);
		return OdometryUtils::getAngleFromQuaternation
			(quaternion.normalize(), false);
	#else
		return OdometryUtils::getAngleFromQuaternation(
			actualOdometryPosition.pose.pose.orientation);
	#endif

}

bool MovimentationExecutor::subscribeToTopics() {
	ROS_INFO("Subscribing to topics");
	return addSubscribedTopic<const common::Move_robot::ConstPtr&,
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
	return addPublisherClient<geometry_msgs::Twist>(
		   nodeHandler, velTopic, true) &&

		   addPublisherClient<std_msgs::Float32>(
		   nodeHandler, errorTopic, true);
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
	const common::Move_robot::ConstPtr& targetPositionPointer) {
		ROS_DEBUG("Received target position x:%f y:%f ",
			targetPositionPointer->x,targetPositionPointer->y);
		targetPosition.x = targetPositionPointer->x;
		targetPosition.y = targetPositionPointer->y;
		pointerTargetPosition = &targetPosition;
}

//Main
int main(int argc,char **argv) {
	MovimentationExecutor movimentationExecutor(argc, argv, 0.75);
	try {
		if(movimentationExecutor.subscribeToTopics() &&
			movimentationExecutor.createPublishers()) {
				return movimentationExecutor.runNode();
		} else {
			return movimentationExecutor.shutdownAndExit();
		}
	} catch (std::exception &e) {
		return movimentationExecutor.shutdownAndExit(e);
	}
}