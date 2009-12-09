/*
 * PassCommand.cpp
 *
 *  Created on: Dec 9, 2009
 *      Author: Philip Rogers
 */

#include <gameplay/plays/test_plays/PassCommand.hpp>

PassCommand::PassCommand(CommandType cT, Robot* a, Robot* b, Point* bPI, Point* bGP, Point* aPI, Point* aDI, Point* aPF, Point* aDF, double time) : commandType(cT), robotA(a), robotB(b), time(time) {
	if(bPI != NULL){ballPosInit.x = bPI->x; ballPosInit.y = bPI->y;}
	if(bGP != NULL){ballGoalPos.x = bGP->x; ballGoalPos.y = bGP->y;}
	if(aPI != NULL){robotAPosInit.x = aPI->x; robotAPosInit.y = aPI->y;}
	if(aDI != NULL){robotADirInit.x = aDI->x; robotADirInit.y = aDI->y;}
	if(aPF != NULL){robotAPosFinal.x = aPF->x; robotAPosFinal.y = aPF->y;}
	if(aDF != NULL){robotADirFinal.x = aDF->x; robotADirFinal.y = aDF->y;}
}

// convenience constructor for commandType, robotA, robotAPosInit, robotADirInit, robotAPosFinal, robotADirFinal
PassCommand::PassCommand(CommandType cT, Robot* a, Point aPI, Point aDI, Point aPF, Point aDF, double time) :
	commandType(cT), robotA(a), robotB(NULL), robotAPosInit(aPI), robotADirInit(aDI),
	robotAPosFinal(aPF), robotADirFinal(aDF), time(time) {};

PassCommand::~PassCommand() {}

ostream& operator<<(ostream& out, const PassCommand &passCommand){
	out << "{ commandType:" << passCommand.commandType;
	if(passCommand.robotA != NULL)
		out << ", robotA.id():" << passCommand.robotA->id();
	else
		out << ", robotA.id(): null";
	if(passCommand.robotB != NULL)
		out << ", robotB.id():" << passCommand.robotB->id();
	else
		out << ", robotB.id(): null";
	out << ", ballPosInit: (" << passCommand.ballPosInit.x << "," << passCommand.ballPosInit.y << ")";
	out << ", ballGoalPos: (" << passCommand.ballGoalPos.x << "," << passCommand.ballGoalPos.y << ")";
	out << ", robotAPosInit: (" << passCommand.robotAPosInit.x << "," << passCommand.robotAPosInit.y << ")";
	out << ", robotADirInit: (" << passCommand.robotADirInit.x << "," << passCommand.robotADirInit.y << ")";
	out << ", robotAPosFinal: (" << passCommand.robotAPosFinal.x << "," << passCommand.robotAPosFinal.y << ")";
	out << ", robotADirFinal: (" << passCommand.robotADirFinal.x << "," << passCommand.robotADirFinal.y << ")";
	out << ", time: " << passCommand.time;
	return out;
}
