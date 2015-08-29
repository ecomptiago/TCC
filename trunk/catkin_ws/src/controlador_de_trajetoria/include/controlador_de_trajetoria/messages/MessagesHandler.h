/*
 * MessageHandler.h
 *
 *  Created on: Aug 22, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_MESSAGES_MESSAGESHANDLER_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_MESSAGES_MESSAGESHANDLER_H_

#include "std_msgs/String.h"
#include "controlador_de_trajetoria/BaseRosNode.h"

class MessagesHandler :public BaseRosNode{
	public:
		int main(int argc, char **argv);
};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_MESSAGES_MESSAGESHANDLER_H_ */
