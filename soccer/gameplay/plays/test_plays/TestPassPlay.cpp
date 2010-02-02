/*
 * TestPassPlay.cpp
 *
 *  Created on: Nov 29th
 *      Author: Philip Rogers
 *      Author:
 */

#include "TestPassPlay.hpp"

#define TIMEMARGIN 1.5 // seconds a play can deviate from plan before abort
#define ROBOTSUCCESSMARGIN 0.2 // if robot within this region, a move is complete
#define BALLSUCCESSMARGIN 0.2 // if robot within this region, a move is complete

using namespace Geometry2d;
using namespace std;

Gameplay::Plays::TestPassPlay::TestPassPlay(GameplayModule *gameplay)
: Play(gameplay), kicker(gameplay), interceptor(gameplay)/*, optimizer_(gameplay)*/ {
	_passState = Initializing;
	newPassState = true;
	passIndex = 0; // start the index after the first state (0)
	playTime = -1; // set playTime to an invalid time
}

void Gameplay::Plays::TestPassPlay::assign(set<Robot *> &available){
	_passState = Initializing;
	full_available_ = available;
	this->takeAll(available); // assign all robots from available to _robots
}

bool Gameplay::Plays::TestPassPlay::run(){
	if (_passState == Initializing) { // do initialization here
		initializePlan();

		bestPassConfig = initialPlans[0];
		cout << "Plan: " << bestPassConfig << endl;

		_gameplay->_passConfig_primary = &initialPlans[0]; // optimized plan
		_gameplay->_passConfig_secondary = &initialPlans[1]; // initial plan

		// todo: assign the robots that are unused...?

		// goto next state
		_passState = Executing;

		passIndex = 0; // start the index after the first state (0)
		playTime = -1; // set playTime to an invalid time

	}

	if (_passState == Executing) { // perform actual execution
		// sanity check
		if (!allVisible() || !ball().valid)
			return false; // no ball
		if(passIndex >= bestPassConfig.length())
			return false; // invalid state
		if(this->gameState().state != GameState::Playing)
			return false;

		PassState passState = bestPassConfig.getPassState(passIndex);
		PassState nextState = bestPassConfig.getPassState((passIndex+1<bestPassConfig.length()?passIndex+1:passIndex));

		double currentTime = _gameplay->state()->timestamp / 1000000.0;
		if(playTime < 0){playTime = currentTime;}
		if((currentTime-playTime) - passState.timestamp >= TIMEMARGIN){
			cout << "aborting due to invalid plan..." << endl; // abort plan
			_passState = Done;
		}else if(passState.stateType==PassState::INTERMEDIATE){
			passState.robot1->move(passState.robot1Pos);
			passState.robot2->move(passState.robot2Pos);
			if(nextState.stateType==PassState::KICKPASS||nextState.stateType==PassState::KICKGOAL)
				passState.robot1->face(nextState.ballPos,true);
			else
				passState.robot1->face(passState.ballPos,true);
			passState.robot2->face(passState.ballPos,true);

			float robot1GoalPosDist = passState.robot1->pos().distTo(passState.robot1Pos);
			float robot2GoalPosDist = passState.robot2->pos().distTo(passState.robot2Pos);
			if(robot1GoalPosDist < ROBOTSUCCESSMARGIN && robot2GoalPosDist < ROBOTSUCCESSMARGIN){
				newPassState = true; // move complete, move to next state
			}else{newPassState = false;}
		}else if(passState.stateType==PassState::KICKPASS){
			passState.robot2->move(passState.robot2Pos);
			if(newPassState || !kicker.assigned() || kicker.getState()==kicker.Done){
				kicker.assignOne(passState.robot1);
				kicker.targetRobot = passState.robot2;
				kicker.restart();
			}

			float ballPosDist = passState.ballPos.distTo(ball().pos);
			float robot2GoalPosDist = passState.robot2->pos().distTo(passState.robot2Pos);
			bool ballMoved = ballPosDist > BALLSUCCESSMARGIN;
			if(kicker.run() && ballMoved && robot2GoalPosDist < ROBOTSUCCESSMARGIN){
				newPassState = true; // pass complete, move to next state
			}else{newPassState = false;}
		}else if(passState.stateType==PassState::KICKGOAL){
			if(newPassState || !kicker.assigned() || kicker.getState()==kicker.Done){
				cout << "assigning new robot" << endl;
				kicker.assignOne(passState.robot2);
				kicker.targetRobot = NULL;
				kicker.restart();
			}

			float ballPosDist = nextState.ballPos.distTo(ball().pos);
			bool ballGoal = ballPosDist < BALLSUCCESSMARGIN;
			if(kicker.run() && ballGoal){
				newPassState = true; // pass complete, move to next state
			}else{newPassState = false;}
		}else if(passState.stateType==PassState::RECEIVEPASS){
			if(newPassState || !interceptor.assigned()){
				interceptor.assignOne(passState.robot2);
			}

			float ballPosDist = passState.ballPos.distTo(ball().pos);
			float robot2GoalPosDist = passState.robot2->pos().distTo(passState.robot2Pos);
			if(interceptor.run() && ballPosDist < BALLSUCCESSMARGIN && robot2GoalPosDist < ROBOTSUCCESSMARGIN){
				newPassState = true; // receive complete, move to next state
			}else{newPassState = false;}
		}

		if(newPassState){
			passIndex++;
			passState.robot1->resetMotionCommand();
			passState.robot2->resetMotionCommand();
		}

	}

	if(passIndex >= bestPassConfig.length()){
		_passState = Done;
	}

	if(_passState == Done){
		cout << "pass done" << endl;
		_passState = Initializing;
		return false;
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

	//PassConfig * opt = new PassConfig(optimizer_.optimizePlan(initialPlans[idx], false));
	//newConfigs.push_back(opt);
	newConfigs.push_back(new PassConfig(initialPlans[0]));
	newConfigs.push_back(new PassConfig(initialPlans[0]));
	AnalyticPassPlanner::evaluateConfigs(_robots, _gameplay->opp, newConfigs);
	//newConfigs.push_back(new PassConfig(initialPlans[idx]));

	initialPlans.clear();
	initialPlans = newConfigs;

	//for(int i=0; i<(int)initialPlans.size(); i++)
	//	cout << "passConfig(" << i << "): " << initialPlans[i] << endl;
}
