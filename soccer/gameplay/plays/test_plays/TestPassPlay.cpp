/*
 * TestPassPlay.cpp
 *
 *  Created on: Nov 29th
 *      Author: Philip Rogers
 *      Author:
 */

#include "TestPassPlay.hpp"

#define TIMEMARGIN 1.5 // seconds a play can deviate from plan before abort

using namespace Geometry2d;
using namespace std;

Gameplay::Plays::TestPassPlay::TestPassPlay(GameplayModule *gameplay)
: Play(gameplay), kicker(gameplay), optimizer_(gameplay) {
	_passState = Initializing;
	passIndex = 0; // start the index after the first state (0)
	playTime = -1; // set playTime to an invalid time
}

void Gameplay::Plays::TestPassPlay::assign(set<Robot *> &available){
	_passState = Initializing;
	full_available_ = available;
	this->takeAll(available); // assign all robots from available to _robots
}

bool Gameplay::Plays::TestPassPlay::run(){
	if (_passState == Initializing) {
		// do initialization here
		initializePlan();

		bestPassConfig = initialPlans[0];
		cout << "Plan: " << bestPassConfig << endl;

		_gameplay->_passConfig_primary = &initialPlans[0]; // optimized plan
		_gameplay->_passConfig_secondary = &initialPlans[1]; // initial plan


//		vector<Robot> nonPassingRobots;
//		BOOST_FOREACH(Robot *r, _robots)
//			nonPassingRobots.push_back(*r);
//		BOOST_FOREACH(PassState passState, bestPassConfig.passStateVector){
//			Robot *r = passState.robot;
//			if(r != NULL){
//				for (vector<Robot>::iterator it = nonPassingRobots.begin(); it!=nonPassingRobots.end(); ++it) {
//					if(it->id() == r->id()){
//						nonPassingRobots.erase(it);
//						--it;
//					}
//				}
//			}
//		}
//
//		// todo: assign all Robots in nonPassingRobots to defenders
//		nonPassingRobots.clear();

		// goto next state
		_passState = Executing;

		passIndex = 0; // start the index after the first state (0)
		playTime = -1; // set playTime to an invalid time

	} else if (_passState == Executing) {// perform actual execution

		// sanity check
		if (!allVisible() || !ball().valid)
			return false; // no ball
		if(passIndex >= bestPassConfig.length())
			return false; // invalid state
		if(this->gameState().state != GameState::Playing)
			return false;

		// issue move commands
		PassState prev = bestPassConfig.passStateVector[0];
		BOOST_FOREACH(PassState s, bestPassConfig.passStateVector) {
			if (s.stateType == PassState::INTERMEDIATE &&
					prev.stateType == PassState::INTERMEDIATE) {
				s.robot->move(s.robotPos);
			}
			prev = s;
		}

		PassState passState = bestPassConfig.getPassState(passIndex);
		double currentTime = _gameplay->state()->timestamp / 1000000.0;
		if(playTime < 0){playTime = currentTime;}

		//cout << "current time: " << (currentTime-playTime) << " should be less than: " << passState.timeLeaveState << endl;
		if((currentTime-playTime) - passState.timeLeaveState >= TIMEMARGIN){
			cout << "aborting due to invalid plan..." << endl; // abort plan
			_robots.clear();
			_passState = Done;
			return false;
		}

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
					}else{
						kicker.targetRobot = 0;
					}
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

	// perform optimization on the first of the plans
	PassConfigVector newConfigs;

	//find a plan that uses a pass
	size_t idx = 0;
	BOOST_FOREACH(PassConfig cfg, initialPlans) {
		if (cfg.length() > 3) {
			break;
		}
		++idx;
	}

	PassConfig * opt = new PassConfig(optimizer_.optimizePlan(initialPlans[idx], false));
	newConfigs.push_back(opt);
	AnalyticPassPlanner::evaluateConfigs(_robots, _gameplay->opp, newConfigs);
	newConfigs.push_back(new PassConfig(initialPlans[idx]));

	initialPlans.clear();
	initialPlans = newConfigs;

	//for(int i=0; i<(int)initialPlans.size(); i++)
	//	cout << "passConfig(" << i << "): " << initialPlans[i] << endl;
}
