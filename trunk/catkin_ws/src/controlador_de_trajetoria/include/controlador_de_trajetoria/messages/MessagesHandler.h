/*
 * MessageHandler.h
 *
 *  Created on: Aug 22, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_MESSAGES_MESSAGESHANDLER_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_MESSAGES_MESSAGESHANDLER_H_

#include "vector"
#include "memory"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "common/BaseRosNode.h"
#include "common/Position.h"
#include "controlador_de_trajetoria/messages/MessagesHandler.h"
#include "controlador_de_trajetoria/Move_robot_multi_array.h"
#include "controlador_de_trajetoria/Move_robot_service.h"
#include "controlador_de_trajetoria/utils/MovimentationUtils.h"
#include "controlador_de_trajetoria/messages/MoveRobotWrapper.h"

const char* moveRobotAssyncTopic = "Message_handler/move_robot_assync";
const char* targetPositionTopic = "Message_handler/target_position";
const char* targetPositionAchievedTopic = "Message_handler/target_position_achieved";
const char* freeCoordinatesTopic = "Message_handler/free_coordinates";
const char* nextTargetsTopic = "Message_handler/next_targets";
const char* moveRobotSyncService = "Message_handler/move_robot_sync";
const char* timerFreeCoordinates = "freeCoordinatesTimer";
const char* timerNextTargets = "nextTargetsTimer";

//TODO - use only shared_ptr instead of raw pointer (*)
class MessagesHandler :public BaseRosNode{
	private:
		//Attributes
		/**TODO - Pass this attributed to base class
		to be inherited by derivated class. The actual
		problem is that it can not be instantiated before
		calling ros::init*/
		ros::NodeHandle nodeHandler;
		std::vector<MoveRobotWrapper> coordinatesList;
		std::vector<controlador_de_trajetoria::Move_robot> nextTargetsList;
		double wakeUpTime;
		bool arrivedInTargetPosition;
		float freeCoordinatesDelay; //This is in seconds
		float nextTargetsDelay;
		bool isFinalPositionCorrect;
		int idMoveRobotExecuted;
		bool lockMutex;

	public:

		//Constructors
		MessagesHandler(int argc,char **argv, int numberOfCoordinatesToStore,
				double wakeUpTime, float freeCoordinatesDelay, float nextTargetsDelay);

		//Destructor
		virtual ~MessagesHandler() {} ;

		//Methods
		virtual int runNode();
		bool subscribeToTopics();
		bool createPublishers();
		bool createTimers();
		bool createServices();
		void proccessPositionToMoveRobot(
			const controlador_de_trajetoria::Move_robot::ConstPtr& moveRobotPosition);
		void positionAchieved(
			const common::Position::ConstPtr& positionAchieved);
		bool moveRobotSync(
			controlador_de_trajetoria::Move_robot_service::Request& request,
			controlador_de_trajetoria::Move_robot_service::Response& response);
		void publishFreeCoordinates(const ros::TimerEvent& timerEvent);
		void nextTargets(const ros::TimerEvent& timerEvent);
};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_MESSAGES_MESSAGESHANDLER_H_ */
