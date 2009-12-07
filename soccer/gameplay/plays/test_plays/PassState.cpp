/*
 * PassState.cpp
 *
 *  Created on: Nov 29, 2009
 *      Author: Philip Rogers
 */

#include <gameplay/plays/test_plays/PassState.hpp>

PassState::PassState(const Geometry2d::Point* _bP, StateType _sT) : ballPos(_bP->x,_bP->y), controllingRobot((Robot*)NULL), stateType(_sT) {

}

// if there is a controlling robot, the state type is implicitly INTERMEDIATE
PassState::PassState(const Geometry2d::Point* _bP, Robot* _cR) : ballPos(_bP->x,_bP->y), controllingRobot(_cR), stateType(INTERMEDIATE) {

}


PassState::~PassState() {
	// TODO Auto-generated destructor stub
}

ostream& operator<<(ostream& out, const PassState &state){
	if(state.controllingRobot != NULL)
		out << "robot id:" << state.controllingRobot->id();
	else
		out << "no robot";
	out << ", ball position: (" << state.ballPos.x << ", " << state.ballPos.y << ")";
	out << ", timeEnter: " << state.timeEnterState << ", timeLeave: " << state.timeLeaveState;
	return out;
}
