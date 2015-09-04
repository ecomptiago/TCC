#include <controlador_de_trajetoria/messages/MessagesHandler.h>

//Constructors
MessagesHandler::MessagesHandler(int argc, char **argv, int numberOfCoordinatesToStore) :
	BaseRosNode(argc,argv,nodeName) {
	wakeUpTime = 1;
	arrivedInTargetPosition = false;
	coordinatesList.resize(numberOfCoordinatesToStore);
}

//Methods
int MessagesHandler::runNode() {
	ros::Rate rate(1/wakeUpTime);
	while(ros::ok()) {
		if(coordinatesList.size() > 0) {
			if(arrivedInTargetPosition) {
				ROS_INFO("Sending position to position executor");
				if(hasPublisher(targetPositionTopic)) {
					arrivedInTargetPosition = false;
					publisherMap[targetPositionAchievedTopic].publish(coordinatesList.front());
					ROS_DEBUG("Coordinate was sent. Removing from vector");
					coordinatesList.erase(coordinatesList.begin());
				}
			} else {
				ROS_DEBUG("Did not arrived in position yet");
			}
		} else {
			ROS_DEBUG("Have none position to send robot");
		}
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}

bool MessagesHandler::subscribeToTopics() {
	ROS_INFO("Subscribing to topics");
	ros::Subscriber sub = nodeHandler.subscribe(moveRobotAssyncTopic, 1000,
			&MessagesHandler::proccessPositionToMoveRobot,
			this);
	ros::Subscriber sub2 = nodeHandler.subscribe(targetPositionAchievedTopic, 1000,
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
	if(pub) {
		publisherMap[targetPositionTopic] =  pub;
		return true;
	} else {
		ROS_DEBUG("Could not create all publishers");
		return false;
	}
	return true;
}

//Callback
void MessagesHandler::proccessPositionToMoveRobot(
		const controlador_de_trajetoria::Move_robot::ConstPtr& moveRobotPosition) {
	ROS_INFO("Received position to move robot");
	if(coordinatesList.size() != coordinatesList.max_size()) {
		coordinatesList.push_back(moveRobotPosition);
	} else {
		ROS_DEBUG("Vector is full position will be discarded");
	}
}

void MessagesHandler::positionAchieved(
		const controlador_de_trajetoria::Position::ConstPtr& positionAchieved) {
	ROS_INFO("Check if the position was achieved");

	if(positionAchieved->x == coordinatesList.front()->x &&
			positionAchieved->y == coordinatesList.front()->y) {
		ROS_INFO("Position achieved");
		arrivedInTargetPosition = true;
	} else {
		ROS_DEBUG("Position received is not the first in the"
				"vector");
	}
}

//Main
int main(int argc,char **argv) {
	try {
		MessagesHandler messagesHandler(argc,argv,100);

		if(messagesHandler.subscribeToTopics() && messagesHandler.createPublishers()) {
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
