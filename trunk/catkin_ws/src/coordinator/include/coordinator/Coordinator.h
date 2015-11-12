/*
 * Coordinator.h
 *
 *  Created on: Nov 12, 2015
 *      Author: tcoelho
 */

#ifndef SRC_COORDINATOR_H_
#define SRC_COORDINATOR_H_

#include "common/BaseRosNode.h"

const char* nodeName = "Coordinator";

class Coordinator : public BaseRosNode{
public:
	Coordinator(int argc, char **argv);
	virtual ~Coordinator();
};

#endif /* SRC_COORDINATOR_H_ */
