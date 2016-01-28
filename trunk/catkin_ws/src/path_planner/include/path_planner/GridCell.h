/*
 * GridCell.h
 *
 *  Created on: Jan 28, 2016
 *      Author: tcoelho
 */

#ifndef PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_GRIDCELL_H_
#define PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_GRIDCELL_H_

#include "search/CellInterface.h"

class GridCell: public CellInterface{

	private:

		//Attributes
		int position;
		float heuristicValue;
		int costValue;

	public:

		//Constructor
		GridCell();
		GridCell(int position);

		//Destructor
		virtual ~GridCell() {};

		//Getters and setters
		int getCostValue() const;
		void setCostValue(int costValue);
		float getHeuristicValue() const;
		void setHeuristicValue(float heuristicValue);
		int getPosition() const;
		void setPosition(int position);

		//Methods
		virtual bool calculateHeuristicValue(int targetPosition);
};

#endif /* PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_GRIDCELL_H_ */
