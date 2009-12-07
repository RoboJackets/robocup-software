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
	PassState(const Point* ballPos);
	PassState(const Point* ballPos, Robot* controllingRobot);
	virtual ~PassState();
	friend ostream& operator<<(ostream& out, const PassState &state);

	Point ballPos;
	Robot* controllingRobot;
};

#endif /* PASSSTATE_HPP_ */
