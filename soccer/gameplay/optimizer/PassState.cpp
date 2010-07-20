/*
 * PassState.cpp
 *
 *  Created on: Nov 29, 2009
 *      Author: Philip Rogers
 */

#include <gameplay/optimizer/PassState.hpp>

using namespace std;
using namespace Gameplay;
using namespace Geometry2d;

PassState::PassState(Robot* robot1, Robot* robot2,
					const Point &robot1Pos, const Point &robot2Pos,
					const float &robot1Rot, const float &robot2Rot,
					const Point &ballPos, StateType stateType, double timestamp)
: robot1(robot1), robot2(robot2),
  robot1Pos(robot1Pos), robot2Pos(robot2Pos),
  robot1Rot(robot1Rot), robot2Rot(robot2Rot),
  ballPos(ballPos), stateType(stateType), timestamp(timestamp){}

// copy constructor
PassState::PassState(const PassState& s)
: robot1(s.robot1), robot2(s.robot2),
  robot1Pos(s.robot1Pos), robot2Pos(s.robot2Pos),
  robot1Rot(s.robot1Rot), robot2Rot(s.robot2Rot),
  ballPos(s.ballPos), stateType(s.stateType), timestamp(s.timestamp){}

PassState::~PassState() {}

ostream& operator<<(ostream& out, const PassState &state){
	out << "r1.id(" << state.robot1->id() << "), ";
	out << "r1.pos(" << state.robot1Pos.x << "," << state.robot1Pos.y << "), ";
	out << "r1.rot(" << state.robot1Rot << "), ";
	out << "r2.id(" << state.robot2->id() << "), ";
	out << "r2.pos(" << state.robot2Pos.x << "," << state.robot2Pos.y << "), ";
	out << "r2.rot(" << state.robot2Rot << "), ";
	out << "ballPos(" << state.ballPos.x << "," << state.ballPos.y << "), ";
	out << "timestamp(" << state.timestamp << "), ";
	out << "stateType(";
	switch(state.stateType){
		case(PassState::INITIAL): out << "INITIAL"; break;
		case(PassState::KICKPASS) : out << "KICKPASS"; break;
		case(PassState::RECEIVEPASS) : out << "RECEIVEPASS"; break;
		case(PassState::KICKGOAL) : out << "KICKGOAL"; break;
		case(PassState::GOAL): default :out << "GOAL"; break;
	}
	out << ")";
	return out;
}
