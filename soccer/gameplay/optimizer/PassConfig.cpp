/*
 * PassConfig.cpp
 *
 *  Created on: Dec 4, 2009
 *      Author: Philip Rogers
 */

#include <boost/foreach.hpp>
#include "PassConfig.hpp"

using namespace std;
using namespace Packet;

PassConfig::PassConfig() : weight(0.0) { }

PassConfig::PassConfig(const PassConfig& c)
: weight(c.weight){
	// copy over the states
	for (int i=0; i<c.length(); ++i)
		passStateVector.push_back(c.getPassState(i));
}

PassConfig::~PassConfig() {
	passStateVector.clear();
}

void PassConfig::addPassState(const PassState& passState){
	passStateVector.push_back(passState);
}

PassState PassConfig::getPassState(int idx) const {
	assert(idx < (int)passStateVector.size()); // pass state index must exist
	return passStateVector.at(idx);
}

int PassConfig::length() const{
	return passStateVector.size();
}

void PassConfig::setWeight(double w){
	weight = w;
}

ostream& operator<<(ostream& out, const PassConfig &config){
	out << "config weight(" << config.weight << "):" << endl;
	for(int i=0; i<(int)config.passStateVector.size(); i++){
		out << "\t state(" << i << "):" << config.passStateVector.at(i) << endl;
	}
	return out;
}

bool operator<(const PassConfig& lhs, const PassConfig& rhs){
	return (lhs.weight < rhs.weight);
}

void PassConfig::drawConfig(SystemState* sysState, int r, int g, int b) const {
	// parameters for drawing
// 	float ball_radius = 0.05;
// 	float pos_radius = 0.1;

	// Hardcoded printing, assuming
	// Intermediate (init), KickPass, ReceivePass, KickGoal, Intermediate (ball in goal)
	if (passStateVector.size() != 5)
		throw invalid_argument("Incorrect size of passConfig - hardcoded for 5");

	// pull out the states
	const PassState start    = passStateVector[0],
					kickPass = passStateVector[1],
					recPass  = passStateVector[2],
					kickGoal = passStateVector[3],
					end      = passStateVector[4];

#if 0
	// render initial positions
	sysState->debugCircles.push_back(drawCircle(start.robot1Pos, pos_radius, r,g,b));
	sysState->debugCircles.push_back(drawCircle(start.robot2Pos, pos_radius, r,g,b));

	// render kickPass states
	sysState->debugCircles.push_back(drawCircle(kickPass.robot1Pos, pos_radius, r,g,b));
	sysState->debugLines.push_back(drawLine(start.robot1Pos, kickPass.robot1Pos, r,g,b));
	sysState->debugCircles.push_back(drawCircle(kickPass.ballPos, ball_radius, r,g,b));

	// render recPass state
	sysState->debugCircles.push_back(drawCircle(recPass.robot2Pos, pos_radius, r,g,b));
	sysState->debugLines.push_back(drawLine(kickPass.ballPos, recPass.ballPos, r,g,b));
	sysState->debugLines.push_back(drawLine(kickPass.robot2Pos, recPass.robot2Pos, r,g,b));
	sysState->debugCircles.push_back(drawCircle(recPass.ballPos, ball_radius, r,g,b));

	// render kickGoal state
	sysState->debugCircles.push_back(drawCircle(kickGoal.robot2Pos, pos_radius, r,g,b));
	sysState->debugCircles.push_back(drawCircle(kickGoal.ballPos, ball_radius, r,g,b));
	sysState->debugLines.push_back(drawLine(kickGoal.ballPos, end.ballPos, r,g,b));
	sysState->debugLines.push_back(drawLine(recPass.robot2Pos, kickGoal.robot2Pos, r,g,b));

	// render final ball position
	sysState->debugCircles.push_back(drawCircle(end.ballPos, ball_radius, r,g,b));
#endif

	// Code to do arbitrary paths while printing excess information
//	// draw the path of the ball and the robot trajectories
//	int stateNum = 0;
//	PassState prevState;
//	BOOST_FOREACH(PassState state, passStateVector) {
//
//		// draw the ball
//		sysState->debugCircles.push_back(drawCircle(state.ballPos, ball_radius, r,g,b));
//
//		// draw robots
//		sysState->debugCircles.push_back(drawCircle(state.robot1Pos, pos_radius, r,g,b));
//		sysState->debugCircles.push_back(drawCircle(state.robot2Pos, pos_radius, r,g,b));
//
//		if(stateNum > 0){
//			sysState->debugLines.push_back(drawLine(prevState.ballPos, state.ballPos, r,g,b));
//			sysState->debugLines.push_back(drawLine(prevState.robot1Pos, state.robot1Pos, r,g,b));
//			sysState->debugLines.push_back(drawLine(prevState.robot2Pos, state.robot2Pos, r,g,b));
//		}
//		prevState = state;
//		stateNum++;
//	}
}
