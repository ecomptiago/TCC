/*
 * BaseGridCell.h
 *
 *  Created on: Feb 4, 2016
 *      Author: tcoelho
 */

#ifndef PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_SEARCH_BASEGRIDCELL_H_
#define PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_SEARCH_BASEGRIDCELL_H_

#include "GridCellInterface.h"
#include "common/error/MethodNotImplementedError.h"

template<typename T>
class BaseGridCell: public GridCellInterface{

	public:

		//Attributes
		//TODO - This should be private and accessed by getters and setters
		T cost;
		int cellGridPosition;

		//Constructor
		BaseGridCell(){
			this->cellGridPosition = -1;
		}

		BaseGridCell(int cellGridPosition) {
			this->cellGridPosition = cellGridPosition;
		}

		//Destructor
		virtual ~BaseGridCell() {};

		//Methods
		virtual bool calculateCellCost(int initialGridPosition) {
			MethodNotImplementedError error(__func__,"BaseGridCell");
		}

};

#endif /* PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_SEARCH_BASEGRIDCELL_H_ */
