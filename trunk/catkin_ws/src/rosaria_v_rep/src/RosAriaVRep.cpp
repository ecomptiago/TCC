/*
 * RosAriaVRep.cpp
 *
 *  Created on: Sep 19, 2015
 *      Author: tiago
 */

#include "rosaria_v_rep/RosAriaVRep.h"

//Constructors
RosAriaVRep::RosAriaVRep(int argc, char **argv) :
	BaseRosNode(argc, argv, nodeName){
}

//Methods
int RosAriaVRep::runNode() {
	ROS_INFO("Running node");
	if(VRepUtils::getObjectHandle(motorEsquerdoObjectHandleName,nodeHandler,signalObjectMap) &&
		VRepUtils::getObjectHandle(motorDireitoObjectHandleName,nodeHandler,signalObjectMap) &&
		VRepUtils::getObjectHandle(pionnerLxObjectHandleName,nodeHandler,signalObjectMap) &&
		VRepUtils::getObjectHandle(laserObjectHandleName,nodeHandler,signalObjectMap) &&
		VRepUtils::getObjectHandle(laserBodyObjectHandleName,nodeHandler,signalObjectMap)) {
			rosaria_v_rep::simRosEnableSubscriber simRosEnableSubscriber =
				createEnableSubscriber(cmdVelTopic,simros_strmcmd_set_twist_command, -1, -1);
			serviceClientsMap[enableSubscriberService].call(simRosEnableSubscriber);
			if(simRosEnableSubscriber.response.subscriberID == -1) {
				infoFailAndExit(cmdVelTopic);
			}

			rosaria_v_rep::simRosEnablePublisher simRosEnablePublisher =
				createEnablePublisher(poseTopic, simros_strmcmd_get_object_pose,
				signalObjectMap[pionnerLxObjectHandleName], -1, "");
			serviceClientsMap[enablePublisherService].call(simRosEnablePublisher);
			if(simRosEnablePublisher.response.effectiveTopicName.length() == 0) {
				infoFailAndExit(poseTopic);
			}

			rosaria_v_rep::simRosEnablePublisher simRosEnablePublisher2 =
				createEnablePublisher(laserTopic, simros_strmcmd_get_laser_scanner_data,
				signalObjectMap[laserBodyObjectHandleName], -1, laserVRepSignal);

			serviceClientsMap[enablePublisherService].call(simRosEnablePublisher2);
			if(simRosEnablePublisher2.response.effectiveTopicName.length() == 0) {
				infoFailAndExit(laserTopic);
			}

			std_msgs::Bool boolean;
			boolean.data = true;
			if(hasPublisher(motorStateTopic)) {
				publisherMap[motorStateTopic].publish(boolean);
			} else {
				infoFailAndExit(motorStateTopic);
			}

	} else {
		VRepUtils::infoFailgetObjectsAndExit(nodeName);
	}

	ros::spin();
	BaseRosNode::shutdownAndExit(nodeName);
}

void RosAriaVRep::setWheelsVelocity(rosaria_v_rep::simRosSetJointState& simRosSetJointState,
	float leftWheelVelocity, float rightWheelVelocity) {
		simRosSetJointState.request.values.push_back(rightWheelVelocity);
		simRosSetJointState.request.values.push_back(leftWheelVelocity);
}

void RosAriaVRep::stop() {
	ROS_DEBUG("Stopping robot");
	rosaria_v_rep::simRosSetJointState simRosSetJointState =
			createJointState();
	simRosSetJointState.request.values.push_back(0);
	simRosSetJointState.request.values.push_back(0);
	serviceClientsMap[setJointStateService].call(simRosSetJointState);
	if(simRosSetJointState.response.result == -1) {
		ROS_INFO("Could not set velocity to wheels");
	} else {
		ROS_INFO("Wheels velocity set");
	}
}

bool RosAriaVRep::subscribeToTopics() {
	ROS_INFO("Subscribing to topics");
	return addSubscribedTopic<const geometry_msgs::Twist::ConstPtr&, RosAriaVRep>(nodeHandler,cmdVelTopic,
		&RosAriaVRep::receivedTwist,this);
}

bool RosAriaVRep::createServices() {
	ROS_INFO("Creating services");
	if(createServiceClients()) {
		return createServiceServers();
	} else {
		return false;
	}
}

bool RosAriaVRep::createServiceClients() {
	return addServiceClient<rosaria_v_rep::simRosEnablePublisher>(nodeHandler, enablePublisherService) &&

		   addServiceClient<rosaria_v_rep::simRosEnableSubscriber>(nodeHandler, enableSubscriberService) &&

		   addServiceClient<rosaria_v_rep::simRosSetJointState>(nodeHandler, setJointStateService);

}

bool RosAriaVRep::createServiceServers() {
	return addServiceServer<std_srvs::Empty::Request&, std_srvs::Empty::Response&, RosAriaVRep>(nodeHandler,enableMotorService,
		&RosAriaVRep::enableMotorServiceCallback, this) &&

		addServiceServer<std_srvs::Empty::Request&, std_srvs::Empty::Response&, RosAriaVRep>(nodeHandler,disableMotorService,
		&RosAriaVRep::disableMotorServiceCallback, this);
}

bool RosAriaVRep::enableMotorServiceCallback(std_srvs::Empty::Request& request,
	std_srvs::Empty::Response& response) {
		return true;
}

bool RosAriaVRep::disableMotorServiceCallback(std_srvs::Empty::Request& request,
	std_srvs::Empty::Response& response) {
		return true;
}

rosaria_v_rep::simRosEnableSubscriber RosAriaVRep::createEnableSubscriber(const char* topicName,
	int streamCmd, int auxInt1, int auxInt2) {
		rosaria_v_rep::simRosEnableSubscriber simRosEnableSubscriber;
		simRosEnableSubscriber.request.topicName = topicName;
		simRosEnableSubscriber.request.queueSize = defaultQueueSize;
		simRosEnableSubscriber.request.streamCmd = streamCmd;
		simRosEnableSubscriber.request.auxInt1 = auxInt1;
		simRosEnableSubscriber.request.auxInt2 = auxInt2;
		return simRosEnableSubscriber;
}

rosaria_v_rep::simRosEnablePublisher RosAriaVRep::createEnablePublisher(const char* topicName,
	int streamCmd, int auxInt1, int auxInt2, std::string auxString) {
		rosaria_v_rep::simRosEnablePublisher simRosEnablePublisher;
		simRosEnablePublisher.request.topicName = topicName;
		simRosEnablePublisher.request.queueSize = 1;
		simRosEnablePublisher.request.streamCmd = streamCmd;
		simRosEnablePublisher.request.auxInt1 = auxInt1;
		simRosEnablePublisher.request.auxInt2 = auxInt2;
		if(auxString.compare("") != 0) {
			simRosEnablePublisher.request.auxString = auxString;
		}
		return simRosEnablePublisher;
}

int RosAriaVRep::infoFailAndExit(const char* topicName) {
	ROS_INFO("Failed to create %s topic",topicName);
	return BaseRosNode::shutdownAndExit(nodeName);
}

bool RosAriaVRep::createPublishers() {
	ROS_INFO("Creating publishers");
	return addPublisherClient<std_msgs::Bool>(nodeHandler, motorStateTopic, true);
}

rosaria_v_rep::simRosSetJointState RosAriaVRep::createJointState() {
	rosaria_v_rep::simRosSetJointState simRosSetJointState;

	simRosSetJointState.request.handles.push_back(signalObjectMap[motorEsquerdoObjectHandleName]);
	simRosSetJointState.request.handles.push_back(signalObjectMap[motorDireitoObjectHandleName]);
	simRosSetJointState.request.setModes.push_back(2);
	simRosSetJointState.request.setModes.push_back(2);
	return simRosSetJointState;
}

//Callback
void RosAriaVRep::receivedTwist(
	const geometry_msgs::Twist::ConstPtr& twist) {
		ROS_DEBUG("Received twist. Linear x:%f . Angular z:%f", twist->linear.x
			, twist->angular.z);
		float rightWheelVelocity;
		float leftWheelVelocity;
		rosaria_v_rep::simRosSetJointState simRosSetJointState =
			createJointState();
		if(twist->linear.x != 0 || twist->angular.z != 0) {
			calculateWheelsVelocity(rightWheelVelocity, leftWheelVelocity, twist);
			setWheelsVelocity(simRosSetJointState,leftWheelVelocity,
				rightWheelVelocity);
			ROS_DEBUG("Setting velocity %f for left wheel and %f to right wheel",
				simRosSetJointState.request.values.at(1), simRosSetJointState.request.values.at(0));
			serviceClientsMap[setJointStateService].call(simRosSetJointState);
			if(simRosSetJointState.response.result == -1) {
				ROS_INFO("Could not set velocity to wheels");
			} else {
				ROS_INFO("Wheels velocity set");
			}
		} else if(twist->linear.x == 0 && twist->angular.z == 0){
			stop();
		}
}

void RosAriaVRep::calculateWheelsVelocity(float& rightWheelVelocity,
	float& leftWheelVelocity, const geometry_msgs::Twist::ConstPtr& twist) {
		rightWheelVelocity = 0;
		leftWheelVelocity = 0;
		common::simRosGetObjectPose simRosGetObjectPose;
		VRepUtils::getObjectPose(signalObjectMap[pionnerLxObjectHandleName],
			nodeHandler,simRosGetObjectPose);
		if(simRosGetObjectPose.response.result != -1) {
			float response[2];
			float radiusOfWheel = diameterOfWheels / 2;
			float radiusOfWheelDividedByLenght =
				diameterOfWheels / ( 2 * distanceBetweenCenterOfRobotAndWheels);
			float linearEquationMatrix [2] [3] = {
				{radiusOfWheel, radiusOfWheel, twist->linear.x},
				{radiusOfWheelDividedByLenght, -radiusOfWheelDividedByLenght, twist->angular.z}
			};
			ROS_DEBUG("Matrix of equations is: \n"
				"%f %f %f \n "
				"%f %f %f",
				radiusOfWheel, radiusOfWheel,twist->linear.x,
				radiusOfWheelDividedByLenght, -radiusOfWheelDividedByLenght,
				twist->angular.z);
			if(MatrixUtils::applyGaussElimeliminationWithPartialPivotingAlgorithm<float>(linearEquationMatrix[0],
				2, response)) {
				rightWheelVelocity = response[1] * -1;
				leftWheelVelocity = response[0];
			}
		} else {
			ROS_WARN("Could not get pose from object %s", pionnerLxObjectHandleName);
		}
}

float RosAriaVRep::round(float numberToRound) {
	if(-pow(10, -6) < numberToRound && numberToRound < pow(10, -6)) {
		return 0;
	} else {
		return numberToRound;
	}
}

//Main
int main(int argc, char **argv) {
	RosAriaVRep rosAriaVRep(argc,argv);
	try{
		if(rosAriaVRep.createServices() &&
			rosAriaVRep.subscribeToTopics() &&
			rosAriaVRep.createPublishers()) {
				return rosAriaVRep.runNode();
		} else {
			BaseRosNode::shutdownAndExit(nodeName);
		}
	} catch (std::exception &e) {
		BaseRosNode::shutdownAndExit(nodeName);
	}
}
