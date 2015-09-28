/*
 * BaseRosNode.h
 *
 *  Created on: Aug 22, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_CONTROLADOR_DE_TRAJETORIA_BASEROSNODE_H_
#define INCLUDE_CONTROLADOR_DE_TRAJETORIA_BASEROSNODE_H_
#define VREP_SIMULATION

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
		/*TODO - Take a look at TimerManager class, maybe
		*we can remove this map */
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

		/*Methods that need to be implemented here, because of compiling
		*issues. More information at
		*http://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor
		*/
		template <class T>
		bool addServiceClient(ros::NodeHandle nodeHandler,
			char const* serviceClientName) {
				ros::ServiceClient service =
					nodeHandler.serviceClient<T>(serviceClientName);
				if(service) {
					serviceClientsMap[serviceClientName] = service;
					return true;
				} else {
					ROS_DEBUG("Could not create service client to %s",serviceClientName);
					return false;
				}
		}

		template<class MReq, class MRes, class T>
		bool addServiceServer(ros::NodeHandle nodeHandler,
			const char* serviceServerName,bool (T::*serviceCallBack)(MReq ,MRes),
			T* instance){
				ros::ServiceServer server =
					nodeHandler.advertiseService(serviceServerName,serviceCallBack,instance);
				if(server) {
					serviceServersMap[serviceServerName] = server;
					return true;
				} else {
					ROS_DEBUG("Could not create service server %s",serviceServerName);
					return false;
				}
		}

		template<class T>
		bool addPublisherClient(ros::NodeHandle nodeHandler,
			char const* topicName, bool latch) {
				ros::Publisher pub =
					nodeHandler.advertise<T>(topicName, defaultQueueSize,latch);
				if(pub) {
					publisherMap[topicName] = pub;
					return true;
				} else {
					ROS_DEBUG("Could not create publisher %s",topicName);
					return false;
				}
		}

		template <class M, class T>
		bool addSubscribedTopic(ros::NodeHandle nodeHandler, const char* topicName,
			void (T::*topicCallback)(const M), T* instance){
				ros::Subscriber sub =
					nodeHandler.subscribe(topicName,defaultQueueSize,topicCallback,instance);
				if(sub) {
					subscriberMap[topicName] = sub;
					return true;
				} else {
					ROS_DEBUG("Could not subscribe to %s topics", topicName);
					return false;
				}
		}

		template <class M, class T>
		bool addTimer(ros::NodeHandle nodeHandler, float timerDelay, const char* timerName,
			void (T::*timerCallback)(const M), T* instance, bool autostart) {
				ros::Timer timer =
					nodeHandler.createTimer(ros::Duration(timerDelay),
					timerCallback, instance, autostart);
				if(timer) {
					timerMap[timerName] = timer;
					return true;
				} else {
					ROS_DEBUG("Could not create timer %s",timerName);
					return false;
				}
		}

	public:
		//Constructors
		BaseRosNode(int argc, char **argv, std::string nodeName);

		//Destructor
		virtual ~BaseRosNode() {};

		//Methods
		static 	int shutdownAndExit(const char* nodeName);
		virtual int runNode();
		virtual bool subscribeToTopics();
		virtual bool createPublishers();
		virtual bool createTimers();
		virtual bool createServices();
};

#endif /* INCLUDE_CONTROLADOR_DE_TRAJETORIA_BASEROSNODE_H_ */
