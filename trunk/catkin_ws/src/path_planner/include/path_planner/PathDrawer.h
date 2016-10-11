/*
 * PathDrawer.h
 *
 *  Created on: Jul 29, 2016
 *      Author: tcoelho
 */

#ifndef INCLUDE_PATH_PLANNER_PATHDRAWER_H_
#define INCLUDE_PATH_PLANNER_PATHDRAWER_H_

/*
 * PathPlanner.h
 *
 *  Created on: Dec 15, 2015
 *      Author: tiago
 */

#ifndef INCLUDE_PATH_PLANNER_PATHPLANNER_H_
#define INCLUDE_PATH_PLANNER_PATHPLANNER_H_

#include "vector"
#include "stdio.h"
#include "nav_msgs/Path.h"
#include "common/BaseRosNode.h"
#include "common/v_repConst.h"
#include "common/simRosGetObjectPose.h"
#include "common/utils/VRepUtils.h"
#include "common/utils/GridUtils.h"

const char* pathTopic = "/PathDrawer/path3";
const char* pionnerHandle = "Pionner_LX";
const char* timerRobotPath = "robotPathTimer";
const char* timerRobotPosition = "robotPositionTimer";

class PathDrawer : public BaseRosNode {

	private:
		//Atttributes
		ros::NodeHandle nodeHandler;
		std::map<std::string,int32_t> signalObjectMap;
		nav_msgs::Path path;
		float robotPathDelay;
		float robotPositionDelay;

		//Methods
		bool createServiceClients();
		bool createServiceServers();
		int infoFailAndExit(const char* topicName);

	public:
		//Constructor
		PathDrawer(int argc, char **argv,float robotPathDelay,
			float robotPositionDelay);

		//Destructor
		virtual ~PathDrawer() {};

		//Methods
		int runNode();
		bool createServices();
		bool subscribeToTopics();
		bool createPublishers();
		bool createTimers();
		void publishPath(const ros::TimerEvent& timerEvent);
		void getPosition(const ros::TimerEvent& timerEvent);
};

#endif /* INCLUDE_PATH_PLANNER_PATHDRAWER_H_ */
#endif /* INCLUDE_PATH_PLANNER_PATHDRAWER_H_ */
