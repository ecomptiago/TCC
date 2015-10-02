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
	if(getObjectHandle(motorEsquerdoObjectHandleName) && getObjectHandle(motorDireitoObjectHandleName) &&
		getObjectHandle(pionnerLxObjectHandleName) && getObjectHandle(laserObjectHandleName) &&
		getObjectHandle(laserBodyObjectHandleName)) {

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
				signalObjectMap[laserBodyObjectHandleName], -1, "pointsPackedX");

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
		ROS_INFO("Failed to get objects from V-Rep simulation. Be sure"
			" that the simulation is running.");
		BaseRosNode::shutdownAndExit(nodeName);
	}

	ros::spin();
	BaseRosNode::shutdownAndExit(nodeName);
}

bool RosAriaVRep::getObjectHandle(const char* objectHandleName) {
	rosaria_v_rep::simRosGetObjectHandle simRosGetObjectHandle;
	simRosGetObjectHandle.request.objectName = objectHandleName;
	serviceClientsMap[getObjectHandleService].call(simRosGetObjectHandle);
	if(simRosGetObjectHandle.response.handle != -1) {
		ROS_DEBUG("Got %d int handle for %s", simRosGetObjectHandle.response.handle,
			objectHandleName);
		signalObjectMap[objectHandleName] = simRosGetObjectHandle.response.handle;
		return true;
	} else {
		return false;
	}
}

void RosAriaVRep::setWheelsVelocity(rosaria_v_rep::simRosSetJointState& simRosSetJointState,
	float leftWheelVelocity, float rightWheelVelocity) {
		simRosSetJointState.request.values.push_back(rightWheelVelocity);
		simRosSetJointState.request.values.push_back(leftWheelVelocity);
}

void RosAriaVRep::stop(rosaria_v_rep::simRosSetJointState& simRosSetJointState) {
	simRosSetJointState.request.values.push_back(0);
	simRosSetJointState.request.values.push_back(0);
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

		   addServiceClient<rosaria_v_rep::simRosGetObjectHandle>(nodeHandler, getObjectHandleService) &&

		   addServiceClient<rosaria_v_rep::simRosEnableSubscriber>(nodeHandler, enableSubscriberService) &&

		   addServiceClient<rosaria_v_rep::simRosSetJointState>(nodeHandler, setJointStateService) &&

		   addServiceClient<rosaria_v_rep::simRosGetObjectPose>(nodeHandler, simRosGetObjectPoseService);
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
	ROS_INFO("Failed to create %s topic",motorStateTopic);
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
		ROS_DEBUG("Received twist. Linear x:%f y:%f z:%f . Angular x:%f y:%f z:%f", twist->linear.x,
			twist->linear.y, twist->linear.z, twist->angular.x, twist->angular.y, twist->angular.z);
		float rightWheelVelocity;
		float leftWheelVelocity;
		rosaria_v_rep::simRosSetJointState simRosSetJointState =
			createJointState();
		if(twist->linear.x != 0 || twist->angular.z) {
			calculateWheelsVelocity(rightWheelVelocity, leftWheelVelocity, twist);
			setWheelsVelocity(simRosSetJointState,leftWheelVelocity,
				rightWheelVelocity);
		} else {
			stop(simRosSetJointState);
		}

		ROS_DEBUG("Setting velocity %f for left wheel and %f to right wheel",
			simRosSetJointState.request.values.at(1), simRosSetJointState.request.values.at(0));
		serviceClientsMap[setJointStateService].call(simRosSetJointState);
		if(simRosSetJointState.response.result == -1) {
			ROS_INFO("Could not set velocity to wheels");
		} else {
			ROS_INFO("Wheels velocity set");
		}
}

void RosAriaVRep::calculateWheelsVelocity(float& rightWheelVelocity,
	float& leftWheelVelocity, const geometry_msgs::Twist::ConstPtr& twist) {
		rightWheelVelocity = 0;
		leftWheelVelocity = 0;
		rosaria_v_rep::simRosGetObjectPose simRosGetObjectPose;
		simRosGetObjectPose.request.handle = signalObjectMap[pionnerLxObjectHandleName];
		simRosGetObjectPose.request.relativeToObjectHandle = -1;
		serviceClientsMap[simRosGetObjectPoseService].call(simRosGetObjectPose);
		if(simRosGetObjectPose.response.result != -1) {
			tf::Quaternion quaternion(
				0, 0, simRosGetObjectPose.response.pose.pose.orientation.z,
				simRosGetObjectPose.response.pose.pose.orientation.w);
			double robotAngle = OdometryUtils::getAngleFromQuaternation(quaternion.normalize());
			ROS_DEBUG("Robot actual angle is %f degrees ", robotAngle);
/**
* For calculate the angle we use the transformation:
*
* [dx/dt] =  [cos(theta)  sin(theta) 0] * [(r*rightWheelVelocity/2)+(r*leftWheelVelocity/2)]
*  dy/dt      -sin(theta) cos(theta) 0                             0
*  dTheta/dt  0           0          1     (r*rightWheelVelocity/2*l)+(r*leftWheelVelocity/2*l)
*
* where dx/dt     = velocity in direction of global axis x
*       dy/dt     = velocity in direction of global axis y
*       dTheta/dt = velocity of rotation related (angular velocity)
*       theta     = angle between x global axis and robot x axis
*       r         = wheel diameter
*       l         = distance between the center of the robot and wheels
*
* This transformation is in chapter 3 from book Introduction to Autonomous Mobile Robots
* of Siegwart Nourbakhsh.
*
* Using this transformation and the additional equations
*
* dr/dt = dx/dt + dy/dt
*
* where dr/dt = linear velocity, we have the linear system:
*
* (((cos(theta) * r) / 2) * rightWheelVelocity) + (((cos(theta) * r) / 2) * leftWheelVelocity) - (dx/dt)           = 0
* (((sin(theta) * r) / 2) * rightWheelVelocity) + (((sin(theta) * r) / 2) * leftWheelVelocity)           - (dy/dt) = 0
* (( r * (2 * l)) * rightWheelVelocity)         - (( r * (2 * l)) * leftWheelVelocity)                             = dTheta/dt
* 																							   + (dx/dt) + (dy/dt) = dr/dt
*/
			ROS_INFO("Calculating wheels velocity");
			float response [4];
			float equationMatrixElementCos   = round(((cos(robotAngle * M_PI /180) * diameterOfWheels) / 2));
			float equationMatrixElementSin   = round(((sin(robotAngle * M_PI /180) * diameterOfWheels) / 2));
			float equationMatrixElementConst = round(diameterOfWheels / ( 2 * distanceBetweenCenterOfRobotAndWheels));
			float linearEquationMatrix [4] [5] = {
				{equationMatrixElementCos,    equationMatrixElementCos,   -1,  0, 0},
				{equationMatrixElementSin,    equationMatrixElementSin,    0, -1, 0},
				{equationMatrixElementConst, -equationMatrixElementConst,  0,  0, twist->angular.z},
				{0                         ,  0                         ,  1,  1, twist->linear.x}
			};
			ROS_DEBUG("Matrix of equations is: \n"
				 "%f %f -1 0 0 \n"
				 "%f %f 0 -1 0 \n"
				 "%f %f 0 0 %f \n"
				 "0 0 1 1 %f \n",
				 equationMatrixElementCos, equationMatrixElementCos,
				 equationMatrixElementSin, equationMatrixElementSin,
				 equationMatrixElementConst, -equationMatrixElementConst,twist->angular.z,
				 twist->linear.x);
			if(MatrixUtils::applyGaussElimeliminationWithPartialPivotingAlgorithm<float>(linearEquationMatrix[0],
				4, response)) {
					ROS_DEBUG("Values for solution are rightWheelVelocity: %f leftWheelVelocity:%f "
						"dx/dt:%f dy/dt:%f", response[0], response[1], response[2], response[3]);
					rightWheelVelocity = response[0] * -1; /*Here we need to multiply by -1 because the
					*wheels does not have the same rotation reference ( left wheel with a positive velocity
					*rotates to left and right wheel with a positive velocity rotates to left)*/
					leftWheelVelocity = response[1];
			} else {
				ROS_WARN("Could not resolve linear system");
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
