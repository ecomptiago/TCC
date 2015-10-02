#include <controlador_de_trajetoria/messages/MessagesHandler.h>

//Constructors
MessagesHandler::MessagesHandler(int argc, char **argv, int numberOfCoordinatesToStore,
	double wakeUpTime, float freeCoordinatesDelay, float nextTargetsDelay) :
	BaseRosNode(argc,argv,nodeName) {
		this->wakeUpTime = wakeUpTime;
		this->freeCoordinatesDelay = freeCoordinatesDelay;
		this->nextTargetsDelay = nextTargetsDelay;
		coordinatesList.resize(numberOfCoordinatesToStore);
		coordinatesList.clear();
		nextTargetsList.resize(numberOfCoordinatesToStore);
		nextTargetsList.clear();
		arrivedInTargetPosition = true;
		isFinalPositionCorrect = true;
		idMoveRobotExecuted = -1;
		lockMutex = false;
}

//Methods
int MessagesHandler::runNode() {
	ros::AsyncSpinner spinner(4);
	spinner.start();
	ros::Rate rate(1/wakeUpTime);
	ROS_INFO("Running node");
	while(ros::ok()) {
		if(coordinatesList.size() > 0) {
			if(arrivedInTargetPosition) {
				ROS_INFO("Sending position to position executor");
				if(hasPublisher(targetPositionTopic)) {
					arrivedInTargetPosition = false;
					publisherMap[targetPositionTopic].
						publish(coordinatesList.front().getMoveRobotObj());
				} else {
					ROS_DEBUG("Could not find topic %s to publish",targetPositionTopic);
				}
			}
		}
		sleepAndSpin(rate);
	}
	return 0;
}

bool MessagesHandler::subscribeToTopics() {
	ROS_INFO("Subscribing to topics");
	return addSubscribedTopic<const controlador_de_trajetoria::Move_robot::ConstPtr&, MessagesHandler>(
		   nodeHandler,moveRobotAssyncTopic, &MessagesHandler::proccessPositionToMoveRobot,this) &&

		   addSubscribedTopic<const controlador_de_trajetoria::Position::ConstPtr&, MessagesHandler>(
		   nodeHandler,targetPositionAchievedTopic, &MessagesHandler::positionAchieved,this);
}

bool MessagesHandler::createPublishers() {
	ROS_INFO("Creating publishers");
	return addPublisherClient<controlador_de_trajetoria::Move_robot>(nodeHandler, targetPositionTopic, false) &&

		   addPublisherClient<std_msgs::Int32>(nodeHandler, freeCoordinatesTopic, false) &&

		   addPublisherClient<controlador_de_trajetoria::Move_robot_multi_array>(nodeHandler, nextTargetsTopic, false);
}

bool MessagesHandler::createTimers() {
	ROS_INFO("Creating timers");
	return addTimer<const ros::TimerEvent&, MessagesHandler>(nodeHandler,  freeCoordinatesDelay, timerFreeCoordinates,
		   &MessagesHandler::publishFreeCoordinates, this, false) &&

		   addTimer<const  ros::TimerEvent&, MessagesHandler>(nodeHandler, nextTargetsDelay, timerNextTargets,
		   &MessagesHandler::nextTargets, this, false);

}

bool MessagesHandler::createServices() {
	ROS_INFO("Creating services");
	return addServiceServer<controlador_de_trajetoria::Move_robot_service::Request&, controlador_de_trajetoria::Move_robot_service::Response&,
		   MessagesHandler>(nodeHandler,moveRobotSyncService, &MessagesHandler::moveRobotSync, this);
}

//Callback
void MessagesHandler::proccessPositionToMoveRobot(
	const controlador_de_trajetoria::Move_robot::ConstPtr& moveRobotPosition) {
		ROS_INFO("Received position to move robot");
		if(coordinatesList.size() != coordinatesList.max_size()) {
			ROS_DEBUG("Adding position x:%f y:%f to vector",
				moveRobotPosition->x, moveRobotPosition->y);
			coordinatesList.push_back(MoveRobotWrapper(moveRobotPosition));
			nextTargetsList.push_back(*moveRobotPosition);
		} else {
			ROS_DEBUG("Vector is full position will be discarded");
		}
}

void MessagesHandler::positionAchieved(
	const controlador_de_trajetoria::Position::ConstPtr& positionAchieved) {
		ROS_INFO("Checking if the position was achieved");
		ROS_DEBUG("Position achieved is x:%f y:%f",positionAchieved->x,
			positionAchieved->y);
		if(positionAchieved->x > coordinatesList.front().getMoveRobotObj().x - positionErrorMargin &&
			positionAchieved->x < coordinatesList.front().getMoveRobotObj().x + positionErrorMargin &&
			positionAchieved->y > coordinatesList.front().getMoveRobotObj().y - positionErrorMargin &&
			positionAchieved->y < coordinatesList.front().getMoveRobotObj().y + positionErrorMargin) {
				ROS_INFO("Position achieved");
				isFinalPositionCorrect = true;
				idMoveRobotExecuted = coordinatesList.front().getId();
				nextTargetsList.erase(nextTargetsList.begin());
				coordinatesList.erase(coordinatesList.begin());
		} else {
			ROS_WARN("Position not achieved .Erasing vector");
			coordinatesList.clear();
		}
		arrivedInTargetPosition = true;
}

void MessagesHandler::publishFreeCoordinates(
	const ros::TimerEvent& timerEvent) {
		if(hasPublisher(freeCoordinatesTopic)) {
			std_msgs::Int32 freeCoordinates;
			freeCoordinates.data = coordinatesList.capacity() - coordinatesList.size();
			if(hasPublisher(freeCoordinatesTopic)) {
				publisherMap[freeCoordinatesTopic].publish(freeCoordinates);
			}
		}
}

void MessagesHandler::nextTargets(
	const ros::TimerEvent& timerEvent) {
		if(hasPublisher(nextTargetsTopic)) {
			controlador_de_trajetoria::Move_robot_multi_array nextMoviments;
			nextMoviments.positionsToMoveRobot = nextTargetsList;
			if(hasPublisher(nextTargetsTopic)) {
				publisherMap[nextTargetsTopic].publish(nextMoviments);
			}
		}
}

/*	TODO- Implementation to service move_robot_sync here we need to create a
*  real MUTEX make this service works.Actually only one client can call the service
*  because there is no way to block others threads from enter at this method.
*  The correct way is all the others threads stayed blocked until the thread that
*  is using this code exit.
*/
bool MessagesHandler::moveRobotSync(
	controlador_de_trajetoria::Move_robot_service::Request& request,
	controlador_de_trajetoria::Move_robot_service::Response& response) {
		if(lockMutex) {
			response.serviceIsLocked = true;
			response.arrivedInPosition = false;
			return true;
		}
		ROS_INFO("Received position to move robot");
		ROS_DEBUG("Calling proccessPositionToMoveRobot with position "
			"x:%f y:%f",request.moveRobotPosition.x, request.moveRobotPosition.y);
		MoveRobotWrapper move(request.moveRobotPosition);
		isFinalPositionCorrect = false;
		coordinatesList.push_back(move);
		nextTargetsList.push_back(request.moveRobotPosition);
		lockMutex = true;
		int moveId = move.getId();
		while(idMoveRobotExecuted != moveId) {
			if(coordinatesList.size() == 0) {
				break;
			}
			sleepAndSpin(100);
		}
		lockMutex = false;
		ROS_INFO("Sending response to service caller");
		response.arrivedInPosition = isFinalPositionCorrect;
		response.serviceIsLocked = false;
		idMoveRobotExecuted = -1;
		return true;
}

//Main
int main(int argc,char **argv) {
	try {
		MessagesHandler messagesHandler(argc, argv, 100, 1, 1, 1);
		if(messagesHandler.subscribeToTopics() &&
			messagesHandler.createPublishers() &&
			messagesHandler.createTimers() &&
			messagesHandler.createServices()) {
				return messagesHandler.runNode();
		} else {
			BaseRosNode::shutdownAndExit(nodeName);
		}
	} catch (std::exception &e) {
		BaseRosNode::shutdownAndExit(nodeName);
	}
}
