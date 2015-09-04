/*
 * MessageHandler.h
 *
 *  Created on: Aug 22, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_MESSAGES_MESSAGESHANDLER_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_MESSAGES_MESSAGESHANDLER_H_

#include "vector"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "controlador_de_trajetoria/BaseRosNode.h"
#include "controlador_de_trajetoria/Move_robot.h"
#include "controlador_de_trajetoria/Position.h"


const char* moveRobotAssyncTopic = "Message_handler/move_robot_assync";
const char* targetPositionTopic = "Message_handler/target_position";
const char* targetPositionAchievedTopic = "Message_handler/target_position_achieved";
const char* freeCoordinatesTopic = "Message_handler/free_coordinates";
const char* nodeName = "Message_handler";
const char* timerFreeCoordinates = "freeCoordinatesTimer";

class MessagesHandler :public BaseRosNode{
	private:
		//Attributes
		/**TODO - Pass this attributed to base class
		to be inherited by derivated class. The actual
		problem is that it can not be instantied before
		calling ros::init*/
		ros::NodeHandle nodeHandler;
		std::vector<controlador_de_trajetoria::Move_robot::ConstPtr> coordinatesList;
		double wakeUpTime;
		bool arrivedInTargetPosition;
		float freeCoordinatesDelay; //This is in seconds

	public:
		//Constructors
		MessagesHandler(int argc,char **argv, int numberOfCoordinatesToStore,
				double wakeUpTime, float freeCoordinatesDelay);

		//Destructor
		virtual ~MessagesHandler() {} ;

		//Methods
		virtual int runNode();
		bool subscribeToTopics();
		bool createPublishers();
		bool createTimers();
		void proccessPositionToMoveRobot(
				const controlador_de_trajetoria::Move_robot::ConstPtr& moveRobotPosition);
		void positionAchieved(
				const controlador_de_trajetoria::Position::ConstPtr& positionAchieved);
		void publishFreeCoordinates(const ros::TimerEvent& timerEvent);

};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_MESSAGES_MESSAGESHANDLER_H_ */
