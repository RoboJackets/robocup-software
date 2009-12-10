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
	enum StateType{INITIAL,INTERMEDIATE,GOAL};

	// alternate constructors
	PassState() {} // default - don't read data from this
	PassState(const Point &ballPos, StateType stateType);
	PassState(const Point &ballPos, Robot* robot); // Intermediate, no final robot pos
	PassState(const Point &ballPos, Robot* robot, const Point &robotPos); // Intermediate, with final robot pos

	/// copy constructor
	PassState(const PassState& s);

	virtual ~PassState();
	friend ostream& operator<<(ostream& out, const PassState &state);

	Point ballPos;         // initial position of ball when entering this state
	Robot* robot;
	Point robotPos;        // position of robot when ball leaves this state.
	StateType stateType;
	double timeEnterState; // timestamp when the ball gets to this state (timestamp when ball leaves previous state + travel time)
	double timeLeaveState; // timestamp when the ball leaves this state (includes robot maneuvering)
};

#endif /* PASSSTATE_HPP_ */
