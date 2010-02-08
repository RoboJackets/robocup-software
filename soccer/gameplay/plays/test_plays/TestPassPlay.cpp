/*
 * TestPassPlay.cpp
 *
 *  Created on: Nov 29th
 *      Author: Philip Rogers
 *      Author:
 */

#include "TestPassPlay.hpp"

#define TIMEMARGIN 3.5 // seconds a play can deviate from plan before abort
#define ROBOTSUCCESSMARGIN 0.05 // if robot within this region, a move is complete
#define ROBOTKICKSUCCESSMARGIN 0.05 // if kicking robot within this region, proceed to kick
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

	// make sure robots that are available are sufficient to create a pass
	int numVisible = 0;
	BOOST_FOREACH(Robot *r, available){if(r->visible()){numVisible++;}}
	if(available.size() < 2 || numVisible < 2){
		cout << "Not enough robots to plan with." << endl;
		_passState = Done;
	}
}

bool Gameplay::Plays::TestPassPlay::run(){
	if (_passState == Initializing) { // do initialization here
		initializePlan();

		bestPassConfig = initialPlans[0];
		cout << "Plan: " << bestPassConfig << endl;

		_gameplay->_passConfig_primary = &initialPlans[0]; // optimized plan
		_gameplay->_passConfig_secondary = &initialPlans[1]; // initial plan

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
		}else if(passState.stateType == PassState::INTERMEDIATE){
			if(nextState.stateType == PassState::INTERMEDIATE){
				passState.robot1->face(nextState.ballPos,true);
				passState.robot2->face(passState.ballPos,true);
			}else{
				passState.robot1->face(nextState.ballPos,true);
				passState.robot2->face(passState.ballPos,true);
			}
			passState.robot1->move(passState.robot1Pos);
			passState.robot2->move(passState.robot2Pos);

			float robot1GoalPosDist = passState.robot1->pos().distTo(passState.robot1Pos);
			float robot2GoalPosDist = passState.robot2->pos().distTo(passState.robot2Pos);
			if(robot1GoalPosDist < ROBOTSUCCESSMARGIN && robot2GoalPosDist < ROBOTSUCCESSMARGIN){
				newPassState = true; // move complete, move to next state
			}else{newPassState = false;}
		}else if(passState.stateType==PassState::KICKPASS){
			// drive receiver to receive position
			passState.robot2->move(passState.robot2Pos);
			passState.robot2->face(passState.ballPos,true);

			if(newPassState /*|| !kicker.assigned() || kicker.getState()==kicker.Done*/){
				kicker.assignOne(passState.robot1);
				kicker.targetRobot = passState.robot2;
				kicker.restart();
			}

			float ballPosDist = passState.ballPos.distTo(ball().pos);
			float robot2GoalPosDist = passState.robot2->pos().distTo(passState.robot2Pos);
			bool ballMoved = ballPosDist > BALLSUCCESSMARGIN;
			if(!kicker.run() && kicker.getState()==kicker.Done && ballMoved && robot2GoalPosDist < ROBOTSUCCESSMARGIN){
				newPassState = true; // pass complete, move to next state
				passState.robot1->willKick = false; // do not leave robot in willKick state
			}else{newPassState = false;}
		}else if(passState.stateType==PassState::KICKGOAL){
			if(newPassState /*|| !kicker.assigned() || kicker.getState()==kicker.Done*/){
				kicker.assignOne(passState.robot2);
				kicker.targetRobot = NULL;
				kicker.restart();
			}

			float ballPosDist = nextState.ballPos.distTo(ball().pos);
			bool ballGoal = ballPosDist < BALLSUCCESSMARGIN;
			if(!kicker.run() && kicker.getState()==kicker.Done && ballGoal){
				newPassState = true; // pass complete, move to next state
				passState.robot2->willKick = false; // do not leave robot in willKick state
			}else{newPassState = false;}
		}else if(passState.stateType==PassState::RECEIVEPASS){
			newPassState = false;

			// calculate line that the intercept can occur on
			Point shootGoalVec = nextState.ballPos - nextState.robot2Pos;
			Point ballVec = ball().vel;
			Line receiveLine(shootGoalVec * (-100) + nextState.ballPos, shootGoalVec * (100) + nextState.ballPos);
			Line ballLine(ballVec * (-100) + ball().pos, ballVec * (100) + ball().pos);
			Point interceptPoint;

			if(!receiveLine.intersects(ballLine,&interceptPoint)){
				interceptPoint = ball().pos;
			}

			if(ball().vel.mag() < 0.1){ // if ball is too slow, just go get it
				interceptPoint = ball().pos;
			}

			passState.robot2->face(ball().pos);
			passState.robot2->move(interceptPoint);
			passState.robot2->dribble(50);
			//passState.robot2->willKick = true;

			if(passState.robot2->haveBall()){
				if(passState.robot2->vel().mag() > 0.1){
					passState.robot2->move(passState.robot2->pos());
				}else{
					newPassState = true;
				}
			}
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
