/*
 * PassState.cpp
 *
 *  Created on: Nov 29, 2009
 *      Author: Philip Rogers
 */

#include <gameplay/plays/test_plays/PassState.hpp>


PassState::PassState(const Point* _bP, StateType _sT) : ballPos(_bP->x,_bP->y), robot((Robot*)NULL), stateType(_sT) {}

// if there is a robot, the state type is implicitly INTERMEDIATE
PassState::PassState(const Point* _bP, Robot* _cR) : ballPos(_bP->x,_bP->y), robot(_cR), stateType(INTERMEDIATE) {}
PassState::PassState(const Point* _bP, Robot* _cR, Point* _rP) : ballPos(_bP->x,_bP->y), robot(_cR), robotPos(_rP->x,_rP->y), stateType(INTERMEDIATE) {}



PassState::~PassState() {}

ostream& operator<<(ostream& out, const PassState &state){
	if(state.robot != NULL)
		out << "robot id:" << state.robot->id();
	else
		out << "no robot";
	out << ", init ball pos: (" << state.ballPos.x << ", " << state.ballPos.y << ")";
	out << ", timeEnter: " << state.timeEnterState << ", timeLeave: " << state.timeLeaveState;
	out << ", final robot pos: (" << state.robotPos.x << ", " << state.robotPos.y << ")";
	return out;
}
