/*
 * RosAriaVRep.h
 *
 *  Created on: Sep 19, 2015
 *      Author: tiago
 */

#ifndef SRC_ROSARIAVREP_H_
#define SRC_ROSARIAVREP_H_

#include "common/BaseRosNode.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "rosaria_v_rep/ProximitySensorData.h"
#include "rosaria_v_rep/simRosEnablePublisher.h"
#include "rosaria_v_rep/simRosEnableSubscriber.h"
#include "rosaria_v_rep/simRosGetObjectHandle.h"
#include "rosaria_v_rep/simRosSetJointState.h"
#include "rosaria_v_rep/simRosGetObjectPose.h"
#include "rosaria_v_rep/v_repConst.h"

const char* nodeName = "Rosaria_v_rep";
const char* enablePublisherService = "/vrep/simRosEnablePublisher";
const char* enableSubscriberService = "/vrep/simRosEnableSubscriber";
const char* getObjectHandleService = "/vrep/simRosGetObjectHandle";
const char* setJointStateService = "/vrep/simRosSetJointState";
const char* simRosGetObjectPoseService = "/vrep/simRosGetObjectPose";
const char* cmdVelTopic = "/RosAria/cmd_vel";
const char* poseTopic = "/RosAria/pose";
const char* motorStateTopic = "/RosAria/motors_state";
const char* enableMotorService = "/RosAria/enable_motors";
const char* disableMotorService = "/RosAria/disable_motors";
const char* laserTopic = "laser";
const char* motorDireitoObjectHandleName = "Motor_Direito";
const char* motorEsquerdoObjectHandleName = "Motor_Esquerdo";
const char* pionnerLxObjectHandleName = "Pionner_LX";
const char* laserObjectHandleName = "LaserScanner_2D";

class RosAriaVRep : BaseRosNode {

	private:
		//Atttributes
		ros::NodeHandle nodeHandler;
		std::map<std::string,int32_t> signalObjectMap;

		//Methods
		bool getObjectHandleInt(const char* objectHandleName);
		rosaria_v_rep::simRosSetJointState createJointState();
		void rotateLeft(rosaria_v_rep::simRosSetJointState& simRosSetJointState);
		void rotateRight(rosaria_v_rep::simRosSetJointState& simRosSetJointState);
		void moveForward(rosaria_v_rep::simRosSetJointState& simRosSetJointState);
		void moveBackward(rosaria_v_rep::simRosSetJointState& simRosSetJointState);
		void stop(rosaria_v_rep::simRosSetJointState& simRosSetJointState);
		bool createServiceClients();
		bool createServiceServers();
		rosaria_v_rep::simRosEnableSubscriber createEnableSubscriber(const char* topicName,
			int streamCmd, int auxInt1, int auxInt2);
		rosaria_v_rep::simRosEnablePublisher createEnablePublisher(const char* topicName,
			int streamCmd, int auxInt1, int auxInt2, std::string auxString);
		int infoFailAndExit(const char* topicName);

	public:
		//Constructor
		RosAriaVRep(int argc, char **argv);

		//Destructor
		virtual ~RosAriaVRep() {};

		//Methods
		void receivedTwist(
			const geometry_msgs::Twist::ConstPtr& twist);
		bool enableMotorServiceCallback(
			std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		bool disableMotorServiceCallback(
			std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
		int runNode();
		bool createServices();
		bool subscribeToTopics();
		bool createPublishers();
};

#endif /* SRC_ROSARIAVREP_H_ */
