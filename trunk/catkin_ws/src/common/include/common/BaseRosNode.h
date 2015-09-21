/*
 * BaseRosNode.h
 *
 *  Created on: Aug 22, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_BASEROSNODE_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_BASEROSNODE_H_

#include "stdexcept"
#include "map"
#include "unistd.h"
#include "string"
#include "ros/ros.h"
#include "ros/exceptions.h"
#include "RosNodeInterface.h"
#include "error/MethodNotImplementedError.h"

class BaseRosNode : public RosNodeInterface{

	protected:
		//Attributes
		std::map<std::string,ros::Subscriber> subscriberMap;
		std::map<std::string,ros::Publisher> publisherMap;
		//TODO - Take a look at TimerManager class, maybe
		//we can remove this map
		std::map<std::string,ros::Timer> timerMap;
		std::map<std::string,ros::ServiceClient> serviceClientsMap;
		std::map<std::string,ros::ServiceServer> serviceServersMap;
		double angleErrorMargin;
		double positionErrorMargin;
		int defaultQueueSize;

		//Methods
		bool hasPublisher(const char* topicName);
		void sleepAndSpin(double miliSeconds);
		void sleepAndSpin(ros::Rate& rate);

//		template <typename T>
//		bool addServiceClient(ros::NodeHandle nodeHandler,
//			const char* serviceClientName, T serviceMessage);

	public:
		//Constructors
		BaseRosNode(int argc, char **argv, std::string nodeName);

		//Destructor
		//TODO - Delete all pointers to deallocate memory
		virtual ~BaseRosNode() {};

		//Methods
		static 	int shutdownAndExit(const char* nodeName);
		virtual int runNode();

		//This methods can be generic and implemented by the
		//baserosnode using template <class ContainerAllocator>
		virtual bool subscribeToTopics();
		virtual bool createPublishers();
		virtual bool createTimers();
		virtual bool createServices();
};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_BASEROSNODE_H_ */
