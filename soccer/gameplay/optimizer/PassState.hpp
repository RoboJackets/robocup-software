/*
 * PassState.hpp
 *
 *  Created on: Nov 29, 2009
 *      Author: Philip Rogers
 */

#ifndef PASSSTATE_HPP_
#define PASSSTATE_HPP_

#include <iostream>
#include <gameplay/Robot.hpp>

class PassState {
public:
	enum StateType{INITIAL,KICKPASS,RECEIVEPASS,KICKGOAL,GOAL};

	PassState() {} // default - don't read data from this
	PassState(	Gameplay::Robot* robot1, Gameplay::Robot* robot2,
				const Geometry2d::Point &robot1Pos, const Geometry2d::Point &robot2Pos,
				const float &robot1Rot, const float &robot2Rot,
				const Geometry2d::Point &ballPos, StateType stateType, double timestamp);

	// copy constructor
	PassState(const PassState& s);

	virtual ~PassState();
	friend std::ostream& operator<<(std::ostream& out, const PassState &state);

	Gameplay::Robot* robot1, *robot2;
	Geometry2d::Point robot1Pos, robot2Pos;
	float robot1Rot, robot2Rot;
	Geometry2d::Point ballPos;
	StateType stateType; // action taken when leaving state
	double timestamp;
};

#endif /* PASSSTATE_HPP_ */
