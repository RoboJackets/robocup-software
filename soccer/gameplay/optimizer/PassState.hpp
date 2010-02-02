/*
 * PassState.hpp
 *
 *  Created on: Nov 29, 2009
 *      Author: Philip Rogers
 */

#ifndef PASSSTATE_HPP_
#define PASSSTATE_HPP_

#include <iostream>
#include <fstream>
#include <gameplay/Robot.hpp>

using namespace Geometry2d;
using namespace Gameplay;
using std::ostream;
using std::endl;

class PassState {
public:
	enum StateType{INTERMEDIATE,KICKPASS,RECEIVEPASS,KICKGOAL};

	PassState() {} // default - don't read data from this
	PassState(	Robot* robot1, Robot* robot2,
				const Point &robot1Pos, const Point &robot2Pos,
				const float &robot1Rot, const float &robot2Rot,
				const Point &ballPos, StateType stateType, double timestamp);

	// copy constructor
	PassState(const PassState& s);

	virtual ~PassState();
	friend ostream& operator<<(ostream& out, const PassState &state);

	Robot* robot1, *robot2;
	Point robot1Pos, robot2Pos;
	float robot1Rot, robot2Rot;
	Point ballPos;
	StateType stateType; // action taken when leaving state
	double timestamp;
};

#endif /* PASSSTATE_HPP_ */
