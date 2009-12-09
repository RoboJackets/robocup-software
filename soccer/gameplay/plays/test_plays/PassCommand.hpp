/*
 * PassCommand.hpp
 *
 *  Created on: Dec 9, 2009
 *      Author: Philip Rogers
 */

#ifndef PASSCOMMAND_HPP_
#define PASSCOMMAND_HPP_

#include <iostream>
#include <fstream>
#include <gameplay/Robot.hpp>

using std::ostream;
using std::endl;

using namespace Geometry2d;
using namespace Gameplay;

class PassCommand {
public:
	enum CommandType {AIM_TO_ROBOT, AIM_TO_GOAL, GOTO_POSITION, RECEIVE_BALL, KICK};

	// full constructor
	PassCommand(CommandType cT, Robot* a, Robot* b, Point* bPI, Point* bGP, Point* aPI, Point* aDI, Point* aPF, Point* aDF);
	// convenience constructor for commandType, robotA, robotAPosInit, robotADirInit, robotAPosFinal, robotADirFinal
	PassCommand(CommandType cT, Robot* a, Point aPI, Point aDI, Point aPF, Point aDF);
	virtual ~PassCommand();
	friend ostream& operator<<(ostream& out, const PassCommand &passCommand);

	CommandType commandType;
	Robot* robotA;
	Robot* robotB;
	Point ballPosInit;
	Point ballGoalPos;
	Point robotAPosInit;
	Point robotADirInit;
	Point robotAPosFinal;
	Point robotADirFinal;
};

#endif /* PASSCOMMAND_HPP_ */
