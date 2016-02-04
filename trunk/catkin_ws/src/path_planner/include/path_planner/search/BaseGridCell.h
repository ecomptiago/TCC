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
class BaseGridCell: public GridCellInterface<T>{

	private:

		//Attributes
		int position;
		T cost;

	public:

		//Destructor
		virtual ~BaseGridCell() {};

		//Getters and setters
		T getCost() const {
			return cost;
		}

		void setCost(T cost) {
			this->cost = cost;
		}

		int getPosition() const {
			return position;
		}

		void setPosition(int position) {
			this->position = position;
		}

		virtual T calculateCellCost(int position) {
			MethodNotImplementedError error(__func__,"BaseGridCell");
		}
};

#endif /* PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_SEARCH_BASEGRIDCELL_H_ */
