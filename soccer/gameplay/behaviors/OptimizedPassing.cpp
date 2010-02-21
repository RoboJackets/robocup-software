/**
 * Optimized passing behavior - moved from TestPassPlay
 */

#include <QColor>
#include "OptimizedPassing.hpp"

using namespace Geometry2d;
using namespace std;

Gameplay::Behaviors::OptimizedPassing::OptimizedPassing(GameplayModule *gameplay,
		double time_margin,
		double robotsuccess_margin,
		double ballsuccess_margin,
		bool enableOptimization)
: Behavior(gameplay), kicker(gameplay), interceptor(gameplay), time_margin_(time_margin),
  robotsuccess_margin_(robotsuccess_margin), ballsuccess_margin_(ballsuccess_margin),
  _ballHandlingRange(0.2), _ballControlFrames(10),
  analyticPlanner_(gameplay), enableOptimization_(enableOptimization), optimizer_(gameplay) {
	_passState = Initializing;
	newPassState = true;
	passIndex = 0; // start the index after the first state (0)
	playTime = -1; // set playTime to an invalid time
}

void Gameplay::Behaviors::OptimizedPassing::assign(set<Robot *> &available){
	_passState = Initializing;

	// remove non-visible robots
	_robots.clear();
	int numVisible = 0;
	BOOST_FOREACH(Robot *r, available){
		if(!r->visible()){
			available.erase(r);
		}else{
			numVisible++;
		}
	}

	this->takeAll(available); // assign all robots to _robots
	available.clear();

	if(numVisible < 2){
		cout << "Not enough robots to plan with." << endl;
		_passState = Done;
	}
}

bool Gameplay::Behaviors::OptimizedPassing::run(){
	if (_passState == Initializing) { // do initialization here
		if(initializePlan()){
			bestPassConfig = initialPlans[0];
			cout << "Plan: " << bestPassConfig << endl;
			_passState = Executing; // goto next state

			passIndex = 0; // start the index after the first state (0)
			playTime = -1; // set playTime to an invalid time
		}else{
			return false; // no plans could be found
		}
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

		if(!passState.robot2->haveBall()){_ballControlCounter = 0;}; // reset ball counter

		if((currentTime-playTime) - passState.timestamp >= time_margin_){
			cout << "aborting due to invalid plan..." << endl; // abort plan
			_passState = Done;
		}else if(passState.stateType == PassState::INTERMEDIATE){
			passState.robot1->face(ball().pos,true);
			passState.robot2->face(ball().pos,true);
			passState.robot1->move(passState.robot1Pos);
			passState.robot2->move(passState.robot2Pos);

			float robot1GoalPosDist = passState.robot1->pos().distTo(passState.robot1Pos);
			float robot2GoalPosDist = passState.robot2->pos().distTo(passState.robot2Pos);
			if(robot1GoalPosDist < robotsuccess_margin_ && robot2GoalPosDist < robotsuccess_margin_){
				newPassState = true; // move complete, move to next state
			}else{newPassState = false;}
		}else if(passState.stateType==PassState::KICKPASS){
			// drive receiver to receive position
			passState.robot2->move(passState.robot2Pos);
			passState.robot2->face(ball().pos,true);

			if(newPassState /*|| !kicker.assigned() || kicker.getState()==kicker.Done*/){
				kicker.assignOne(passState.robot1);
				kicker.targetRobot = passState.robot2;
				kicker.restart();
			}

			float ballPosDist = passState.ballPos.distTo(ball().pos);
			float robot2GoalPosDist = passState.robot2->pos().distTo(passState.robot2Pos);
			bool ballMoved = ballPosDist > ballsuccess_margin_;
			if(!kicker.run() && kicker.getState()==kicker.Done && ballMoved && robot2GoalPosDist < robotsuccess_margin_){
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
			bool ballGoal = ballPosDist < ballsuccess_margin_;
			if(!kicker.run() && kicker.getState()==kicker.Done && ballGoal){
				newPassState = true; // pass complete, move to next state
				passState.robot2->willKick = false; // do not leave robot in willKick state
			}else{
				//if(kicker.getState()==kicker.Done){kicker.restart();}
				newPassState = false;
			}
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

			if(ball().vel.mag() < 0.9){ // if ball is too slow, just go get it
				interceptPoint = ball().pos;
			}

			// scale velocity due to range
			if (interceptPoint.distTo(passState.robot2->pos()) <= _ballHandlingRange){
				passState.robot2->setVScale(_ballHandlingScale);
			}

			passState.robot2->face(ball().pos);
			passState.robot2->move(interceptPoint);
			passState.robot2->dribble(50);
			passState.robot2->willKick = true;

			if(passState.robot2->haveBall()){
				_ballControlCounter++;
				if(_ballControlCounter > _ballControlFrames){
					newPassState = true;
					// make sure to reset vscale
					passState.robot1->setVScale(1.0);
					passState.robot2->setVScale(1.0);
				}else{
					passState.robot2->move(passState.robot2->pos());
				}
			}else{
				_ballControlCounter = 0; // reset counter
			}
		}

		if(newPassState){
			passIndex++;
			passState.robot1->resetMotionCommand();
			passState.robot2->resetMotionCommand();
		}

	}

	// perform rendering
	if (initialPlans.size() > 1) {
		QColor optC = Qt::cyan;
		QColor initC = Qt::darkCyan;
		initialPlans[0].drawConfig(gameplay()->state(), optC.red(), optC.green(), optC.blue());
		initialPlans[1].drawConfig(gameplay()->state(), initC.red(), initC.green(), initC.blue());
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

bool Gameplay::Behaviors::OptimizedPassing::initializePlan(){
	initialPlans.clear();

	if(_robots.size() < 2)
		return false; // not enough robots to play with

	analyticPlanner_.generateAllConfigs(ball().pos,_robots,initialPlans);

	if(initialPlans.size() == 0)
		return false; // failed to generate plan

	analyticPlanner_.evaluateConfigs(_robots,_gameplay->opp,initialPlans);

	if(initialPlans.size() == 0)
		return false; // no plans left after evaluation

	// perform optimization on the first of the plans
	AnalyticPassPlanner::PassConfigVector newConfigs;

	// optimize a plan, and then put both before and after in the ready location for render
	if (enableOptimization_) {
		PassConfig * opt = new PassConfig(optimizer_.optimizePlan(initialPlans[0], false));
		newConfigs.push_back(opt);
	} else {
		newConfigs.push_back(new PassConfig(initialPlans[0]));
	}
	newConfigs.push_back(new PassConfig(initialPlans[0])); // push in original

	// reevaluate the configs
	analyticPlanner_.evaluateConfigs(_robots, _gameplay->opp, newConfigs);

	//newConfigs.push_back(new PassConfig(initialPlans[idx]));
	//cout << "Constructed both optimized and non-optimized plan" << endl;

	initialPlans.clear();
	initialPlans = newConfigs;

	return true;

	//for(int i=0; i<(int)initialPlans.size(); i++)
	//	cout << "passConfig(" << i << "): " << initialPlans[i] << endl;
}
