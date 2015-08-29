/*
 * PositionHandler.h
 *
 *  Created on: Aug 25, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_POSITION_POSITIONHANDLER_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_POSITION_POSITIONHANDLER_H_

#include "std_msgs/String.h"
#include "controlador_de_trajetoria/BaseRosNode.h"

class PositionHandler :public BaseRosNode{
	private:
		//Attributes
		/**TODO - Pass this attributed to base class
		to be inherited by derivated class. The actual
		problem is that it can not be instantied before
		calling ros::init*/
		ros::NodeHandle nodeHandler;

	public:
		//Constructors
		PositionHandler(int argc,char **argv);

		//Destructor
		virtual ~PositionHandler() {} ;

		//Getters and setters
		virtual const std::string getNodeName();
		ros::NodeHandle getNodeHandler() const;

		//Methods
		virtual int runNode();
		void callback(const std_msgs::String::ConstPtr& message);
		//TODO - Make it inherited by BaseRosNode class
		bool subscribeToTopics();

};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_POSITION_POSITIONHANDLER_H_ */
