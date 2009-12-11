/*
 * TestPassPlay.cpp
 *
 *  Created on: Nov 29th
 *      Author: Philip Rogers
 *      Author:
 */

#include "TestPassPlay.hpp"

using namespace Geometry2d;
using namespace std;

Gameplay::Plays::TestPassPlay::TestPassPlay(GameplayModule *gameplay) : Play(gameplay), kicker(gameplay) {
	_passState = Initializing;
}

void Gameplay::Plays::TestPassPlay::assign(set<Robot *> &available){
	_passState = Initializing;
	this->takeAll(available); // assign all robots from available to _robots
	initializePlan();

	bestPassConfig = initialPlans[0];

	_gameplay->_passConfig_primary = &initialPlans[0];
	//_gameplay->_passConfig_secondary = &initialPlans[1];


	vector<Robot> nonPassingRobots;
	BOOST_FOREACH(Robot *r, _robots)
		nonPassingRobots.push_back(*r);
	BOOST_FOREACH(PassState passState, bestPassConfig.passStateVector){
		Robot *r = passState.robot;
		if(r != NULL){
			for (vector<Robot>::iterator it = nonPassingRobots.begin(); it!=nonPassingRobots.end(); ++it) {
				if(it->id() == r->id()){
					nonPassingRobots.erase(it);
					--it;
				}
			}
		}
	}

	// todo: assign all Robots in nonPassingRobots to defenders

	nonPassingRobots.clear();

	_passState = Optimizing;

	_passState = Executing;

	passIndex = 0; // start the index after the first state (0)
}

bool Gameplay::Plays::TestPassPlay::run(){
	if(passIndex >= bestPassConfig.length())
		return false;
	PassState passState = bestPassConfig.getPassState(passIndex);

	if(passState.stateType==PassState::GOAL){
		_robots.clear();
		_passState = Done;
		return false;
	}else{
		if(passState.stateType==PassState::INITIAL || !kicker.run()){
			if(++passIndex >= bestPassConfig.length()){return false;}

			passState = bestPassConfig.getPassState(passIndex);
			if(passState.stateType == PassState::INTERMEDIATE){
				kicker.assignOne(passState.robot);
				PassState nextPassState = bestPassConfig.getPassState(passIndex+1);
				if(nextPassState.stateType == PassState::INTERMEDIATE){
					kicker.targetRobot = nextPassState.robot;
					nextPassState.robot->move(nextPassState.robotPos);
				}else{
					kicker.targetRobot = 0;
				}
			}
		}
	}
	return true;
}

void Gameplay::Plays::TestPassPlay::initializePlan(){
	initialPlans.clear();
	AnalyticPassPlanner::generateAllConfigs(ball().pos,_robots,initialPlans);
	AnalyticPassPlanner::evaluateConfigs(_robots,_gameplay->opp,initialPlans);

	for(int i=0; i<(int)initialPlans.size(); i++)
		cout << "passConfig(" << i << "): " << initialPlans[i] << endl;
}
