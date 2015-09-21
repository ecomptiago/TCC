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
	if(getObjectHandleInt(motorEsquerdoObjectHandleName) && getObjectHandleInt(motorDireitoObjectHandleName) &&
		getObjectHandleInt(pionnerLxObjectHandleName) && getObjectHandleInt(laserObjectHandleName)) {
			rosaria_v_rep::simRosEnableSubscriber simRosEnableSubscriber;
			simRosEnableSubscriber.request.topicName = motorTopic;
			simRosEnableSubscriber.request.queueSize = 1000;
			simRosEnableSubscriber.request.streamCmd = simros_strmcmd_set_twist_command;
			simRosEnableSubscriber.request.auxInt1 = -1;
			simRosEnableSubscriber.request.auxInt2 = -1;
			serviceClientsMap[enableSubscriberService].call(simRosEnableSubscriber);
			if(simRosEnableSubscriber.response.subscriberID == -1) {
				return 0;
			}

			rosaria_v_rep::simRosEnablePublisher simRosEnablePublisher;
			simRosEnablePublisher.request.topicName = poseTopic;
			simRosEnablePublisher.request.queueSize = 1000;
			simRosEnablePublisher.request.streamCmd = simros_strmcmd_get_object_pose;
			simRosEnablePublisher.request.auxInt1 = signalObjectMap[pionnerLxObjectHandleName];
			simRosEnablePublisher.request.auxInt2 = sim_handle_parent;
			serviceClientsMap[enablePublisherService].call(simRosEnablePublisher);
			if(simRosEnablePublisher.response.effectiveTopicName.length() == 0) {
				return 0;
			}

			rosaria_v_rep::simRosEnablePublisher simRosEnablePublisher2;
			simRosEnablePublisher2.request.topicName = laserTopic;
			simRosEnablePublisher2.request.queueSize = 1000;
			simRosEnablePublisher2.request.streamCmd = simros_strmcmd_get_laser_scanner_data;
			simRosEnablePublisher2.request.auxInt1 = signalObjectMap[laserObjectHandleName];
			simRosEnablePublisher2.request.auxInt2 = -1;
			simRosEnablePublisher2.request.auxString = "mySignal";
			serviceClientsMap[enablePublisherService].call(simRosEnablePublisher2);
			if(simRosEnablePublisher2.response.effectiveTopicName.length() == 0) {
				return 0;
			}

	} else {
		return 0;
	}

	ros::spin();
	return 0;
}

bool RosAriaVRep::getObjectHandleInt(const char* objectHandleName) {
	rosaria_v_rep::simRosGetObjectHandle simRosGetObjectHandle;
	simRosGetObjectHandle.request.objectName = objectHandleName;
	serviceClientsMap[getObjectHandleService].call(simRosGetObjectHandle);
	if(simRosGetObjectHandle.response.handle != -1) {
		signalObjectMap[objectHandleName] = simRosGetObjectHandle.response.handle;
		return true;
	} else {
		return false;
	}
}

void RosAriaVRep::rotateLeft(rosaria_v_rep::simRosSetJointState& simRosSetJointState) {
	simRosSetJointState.request.values.push_back(1);
	simRosSetJointState.request.values.push_back(1);
}

void RosAriaVRep::rotateRight(rosaria_v_rep::simRosSetJointState& simRosSetJointState) {
	simRosSetJointState.request.values.push_back(-1);
	simRosSetJointState.request.values.push_back(-1);
}

void RosAriaVRep::moveForward(rosaria_v_rep::simRosSetJointState& simRosSetJointState) {
	simRosSetJointState.request.values.push_back(-1);
	simRosSetJointState.request.values.push_back(1);
}

void RosAriaVRep::moveBackward(rosaria_v_rep::simRosSetJointState& simRosSetJointState) {
	simRosSetJointState.request.values.push_back(1);
	simRosSetJointState.request.values.push_back(-1);
}

void RosAriaVRep::stop(rosaria_v_rep::simRosSetJointState& simRosSetJointState) {
	simRosSetJointState.request.values.push_back(0);
	simRosSetJointState.request.values.push_back(0);
}

bool RosAriaVRep::subscribeToTopics() {
	ROS_INFO("Subscribing to topics");
	ros::Subscriber sub =
		nodeHandler.subscribe("vrep/motor",1000,&RosAriaVRep::receivedTwist,this);
	if(sub) {
		subscriberMap[motorTopic] = sub;
		return true;
	} else {
		ROS_INFO("Could not subscribe to all topics");
		return false;
	}
}

bool RosAriaVRep::createServices() {
	ROS_INFO("Creating services");
	ros::ServiceClient service =
		nodeHandler.serviceClient<rosaria_v_rep::simRosEnablePublisher>(enablePublisherService);
	ros::ServiceClient service2 =
		nodeHandler.serviceClient<rosaria_v_rep::simRosGetObjectHandle>(getObjectHandleService);
	ros::ServiceClient service3 =
		nodeHandler.serviceClient<rosaria_v_rep::simRosEnableSubscriber>(enableSubscriberService);
	ros::ServiceClient service4 =
			nodeHandler.serviceClient<rosaria_v_rep::simRosSetJointState>(setJointStateService);
	ros::ServiceClient service5 =
				nodeHandler.serviceClient<rosaria_v_rep::simRosGetObjectPose>(simRosGetObjectPoseService);
	if(service && service2 && service3 && service4 && service5) {
		serviceClientsMap[enablePublisherService] = service;
		serviceClientsMap[getObjectHandleService] = service2;
		serviceClientsMap[enableSubscriberService] = service3;
		serviceClientsMap[setJointStateService] = service4;
		serviceClientsMap[simRosGetObjectPoseService] = service5;
		return true;
	} else {
		ROS_INFO("Could not create all services");
		return false;
	}
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
	ROS_INFO("Received message");
	rosaria_v_rep::simRosSetJointState simRosSetJointState =
		createJointState();

	if(twist->angular.z != 0) {
		if(twist->angular.z > 0) {
			rotateLeft(simRosSetJointState);
		} else{
			rotateRight(simRosSetJointState);
		}
	} else if(twist->linear.x != 0) {
		if(twist->linear.x > 0) {
			moveForward(simRosSetJointState);
		} else{
			moveBackward(simRosSetJointState);
		}
	} else {
		stop(simRosSetJointState);
	}

	serviceClientsMap[setJointStateService].call(simRosSetJointState);
	if(simRosSetJointState.response.result == -1) {
		ROS_INFO("Fodeu");
	}
}

//Main
int main(int argc, char **argv) {
	RosAriaVRep rosAriaVRep(argc,argv);
	try{
		if(rosAriaVRep.createServices() &&
			rosAriaVRep.subscribeToTopics()) {
				return rosAriaVRep.runNode();
		} else {
			BaseRosNode::shutdownAndExit(nodeName);
		}
	} catch (std::exception &e) {
		BaseRosNode::shutdownAndExit(nodeName);
	}
}
