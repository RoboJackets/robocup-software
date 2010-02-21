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

Packet::LogFrame::DebugCircle drawCircle(const Geometry2d::Point& center, float radius, int r, int g, int b) {
	Packet::LogFrame::DebugCircle circle;
	circle.center = center;
	circle.radius(radius);
	circle.color[0] = r;
	circle.color[1] = g;
	circle.color[2] = b;
	return circle;
}

Packet::LogFrame::DebugLine drawLine(const Geometry2d::Point& pt1, const Geometry2d::Point& pt2, int r, int g, int b) {
	Packet::LogFrame::DebugLine line;
	line.pt[0] = pt1;
	line.pt[1] = pt2;
	line.color[0] = r;
	line.color[1] = g;
	line.color[2] = b;
	return line;
}

void PassConfig::drawConfig(SystemState* sysState, int r, int g, int b) const {
	// parameters for drawing
	float ball_radius = 0.05;
	float pos_radius = 0.1;

	// draw the path of the ball and the robot trajectories
	int stateNum = 0;
	PassState prevState;
	BOOST_FOREACH(PassState state, passStateVector) {
//		switch (state.stateType) {
//		case PassState::INTERMEDIATE:
//			// if in first state, draw the robot and ball positions
//			// if in the last state, draw the ball
//			break;
//		case PassState::KICKPASS:
//			break;
//		case PassState::RECEIVEPASS:
//			break;
//		case PassState::KICKGOAL:
//			break;
//		}

		// draw the ball
		sysState->debugCircles.push_back(drawCircle(state.ballPos, ball_radius, r,g,b));

		// draw robots
		sysState->debugCircles.push_back(drawCircle(state.robot1Pos, pos_radius, r,g,b));
		sysState->debugCircles.push_back(drawCircle(state.robot2Pos, pos_radius, r,g,b));

		if(stateNum > 0){
			sysState->debugLines.push_back(drawLine(prevState.ballPos, state.ballPos, r,g,b));
			sysState->debugLines.push_back(drawLine(prevState.robot1Pos, state.robot1Pos, r,g,b));
			sysState->debugLines.push_back(drawLine(prevState.robot2Pos, state.robot2Pos, r,g,b));
		}
		prevState = state;
		stateNum++;
	}
}
