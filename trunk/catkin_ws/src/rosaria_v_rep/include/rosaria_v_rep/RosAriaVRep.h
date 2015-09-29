/*
 * RosAriaVRep.h
 *
 *  Created on: Sep 19, 2015
 *      Author: tiago
 */

#ifndef SRC_ROSARIAVREP_H_
#define SRC_ROSARIAVREP_H_

#include "math.h"
#include "common/BaseRosNode.h"
#include "common/utils/OdometryUtils.h"
#include "common/utils/MatrixUtils.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
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
const char* laserBodyObjectHandleName = "LaserScannerBody_2D";
const float distanceBetweenCenterOfRobotAndWheels = 1; //0.49896 / 2; //This is the l of theory
const float diameterOfWheels = 1; //0.14273; // This is the r of theory

class RosAriaVRep : public BaseRosNode {

	private:
		//Atttributes
		ros::NodeHandle nodeHandler;
		std::map<std::string,int32_t> signalObjectMap;

		//Methods
		bool getObjectHandle(const char* objectHandleName);
		rosaria_v_rep::simRosSetJointState createJointState();
		void setWheelsVelocity(rosaria_v_rep::simRosSetJointState& simRosSetJointState,
			float leftWheelVelocity, float rightWheelVelocity);
		void stop(rosaria_v_rep::simRosSetJointState& simRosSetJointState);
		bool createServiceClients();
		bool createServiceServers();
		rosaria_v_rep::simRosEnableSubscriber createEnableSubscriber(const char* topicName,
			int streamCmd, int auxInt1, int auxInt2);
		rosaria_v_rep::simRosEnablePublisher createEnablePublisher(const char* topicName,
			int streamCmd, int auxInt1, int auxInt2, std::string auxString);
		int infoFailAndExit(const char* topicName);
		void calculateWheelsVelocity(float rightWheelVelocity, float leftWheelVelocity,
			const geometry_msgs::Twist::ConstPtr& twist);

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
