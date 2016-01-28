/*
 * GridCell.h
 *
 *  Created on: Jan 28, 2016
 *      Author: tcoelho
 */

#ifndef PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_GRIDCELL_H_
#define PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_GRIDCELL_H_

class GridCell {
	private:

		//Attributes
		int x;
		int y;

	public:

		//Constructor
		GridCell();

		//Destructor
		virtual ~GridCell() {};

		//Getters and setters
		int getX() const;
		void setX(int x);
		int getY() const;
		void setY(int y);

};

#endif /* PATH_PLANNER_SRC_INCLUDE_PATH_PLANNER_GRIDCELL_H_ */
