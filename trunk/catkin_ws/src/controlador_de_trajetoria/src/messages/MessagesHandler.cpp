#include <controlador_de_trajetoria/messages/MessagesHandler.h>

//Constructors
MessagesHandler::MessagesHandler(int argc, char **argv, int numberOfCoordinatesToStore,
		double wakeUpTime, float freeCoordinatesDelay) :
	BaseRosNode(argc,argv,nodeName) {
	this->wakeUpTime = wakeUpTime;
	this->freeCoordinatesDelay = freeCoordinatesDelay;
	coordinatesList.resize(numberOfCoordinatesToStore);
	coordinatesList.clear();
	arrivedInTargetPosition = true;
}

//Methods
int MessagesHandler::runNode() {
	ros::Rate rate(1/wakeUpTime);
	ROS_INFO("Running node");
	while(ros::ok()) {
		if(coordinatesList.size() > 0) {
			if(arrivedInTargetPosition) {
				ROS_INFO("Sending position to position executor");
				if(hasPublisher(targetPositionTopic)) {
					arrivedInTargetPosition = false;
					publisherMap[targetPositionTopic].publish(coordinatesList.front());
				}
			}
		}
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

bool MessagesHandler::subscribeToTopics() {
	ROS_INFO("Subscribing to topics");
	ros::Subscriber sub =
		nodeHandler.subscribe(moveRobotAssyncTopic, 1000,
		&MessagesHandler::proccessPositionToMoveRobot,
		this);
	ros::Subscriber sub2 =
		nodeHandler.subscribe(targetPositionAchievedTopic, 1000,
		&MessagesHandler::positionAchieved,
		this);
	if(sub && sub2) {
		subscriberMap[moveRobotAssyncTopic] = sub;
		subscriberMap[targetPositionAchievedTopic] = sub2;
		return true;
	} else {
		ROS_DEBUG("Could not subscribe to all topics");
		return false;
	}
}


bool MessagesHandler::createPublishers() {
	ROS_INFO("Creating publishers");
	ros::Publisher pub =
		nodeHandler.advertise<controlador_de_trajetoria::Move_robot>(
		targetPositionTopic, 1000);
	ros::Publisher pub2 =
		nodeHandler.advertise<std_msgs::Int32>(
		freeCoordinatesTopic, 1000);
	ros::Publisher pub3 =
		nodeHandler.advertise<controlador_de_trajetoria::Move_robot_multi_array>(
		nextTargetsTopic, 1000);
	if(pub && pub2) {// && pub3) {
		publisherMap[targetPositionTopic] =  pub;
		publisherMap[freeCoordinatesTopic] =  pub2;
		publisherMap[nextTargetsTopic] =  pub3;
		return true;
	} else {
		ROS_DEBUG("Could not create all publishers");
		return false;
	}
	return true;
}

bool MessagesHandler::createTimers() {
	ROS_INFO("Creating timers");
	ros::Timer timer =
		nodeHandler.createTimer(ros::Duration(freeCoordinatesDelay),
		&MessagesHandler::publishFreeCoordinates,
		this,false);
	ros::Timer timer2 =
		nodeHandler.createTimer(ros::Duration(freeCoordinatesDelay),
		&MessagesHandler::nextTargets,
		this,false);
	if(timer && timer2) {
		timerMap[timerFreeCoordinates] = timer;
		timerMap[timerNextTargets] = timer2;
		return true;
	} else {
		ROS_DEBUG("Could not create all timers");
		return false;
	}
}

//Callback
void MessagesHandler::proccessPositionToMoveRobot(
		const controlador_de_trajetoria::Move_robot::ConstPtr& moveRobotPosition) {
	ROS_INFO("Received position to move robot");
	if(coordinatesList.size() != coordinatesList.max_size()) {
		coordinatesList.push_back(*moveRobotPosition);
	} else {
		ROS_DEBUG("Vector is full position will be discarded");
	}
}

void MessagesHandler::positionAchieved(
		const controlador_de_trajetoria::Position::ConstPtr& positionAchieved) {
	ROS_INFO("Check if the position was achieved");

	if(positionAchieved->x > coordinatesList.front().x - positionErrorMargin &&
	    positionAchieved->x < coordinatesList.front().x + positionErrorMargin &&
	    positionAchieved->y > coordinatesList.front().y - positionErrorMargin &&
	    positionAchieved->y < coordinatesList.front().y + positionErrorMargin) {
			ROS_INFO("Position achieved");
			coordinatesList.erase(coordinatesList.begin());
	} else {
		ROS_WARN("Position received is not the first in the"
			"vector.Erasing vector");
		coordinatesList.clear();
	}
	arrivedInTargetPosition = true;
}

void MessagesHandler::publishFreeCoordinates(
		const ros::TimerEvent& timerEvent) {
	if(hasPublisher(freeCoordinatesTopic)) {
		std_msgs::Int32 freeCoordinates;
		freeCoordinates.data = coordinatesList.capacity() - coordinatesList.size();
		publisherMap[freeCoordinatesTopic].publish(freeCoordinates);
	}
}


void MessagesHandler::nextTargets(
		const ros::TimerEvent& timerEvent) {
	if(hasPublisher(nextTargetsTopic)) {
		controlador_de_trajetoria::Move_robot_multi_array nextMoviments;
		nextMoviments.positionsToMoveRobot = coordinatesList;
		publisherMap[nextTargetsTopic].publish(nextMoviments);
	}
}


//Main
int main(int argc,char **argv) {
	try {
		MessagesHandler messagesHandler(argc,argv,100,1,1);

		if(messagesHandler.subscribeToTopics() &&
				messagesHandler.createPublishers() &&
				messagesHandler.createTimers()) {
			return messagesHandler.runNode();
		} else {
			ros::shutdown();
			return 0;
		}

	} catch (std::exception &e) {
		ros::shutdown();
		return 0;
	}
}
