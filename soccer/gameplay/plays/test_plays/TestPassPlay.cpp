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

Gameplay::Plays::TestPassPlay::TestPassPlay(GameplayModule *gameplay) : Play(gameplay){
	_passState = Initializing;
}

void Gameplay::Plays::TestPassPlay::assign(set<Robot *> &available){
	_passState = Initializing;
	this->takeAll(available); // assign all robots from available to _robots
	initializePlan();

	PassConfig bestPassConfig = initialPlans[0];

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

	// assign all Robots in nonPassingRobots to defenders here
	// todo

	nonPassingRobots.clear();



	//boost::ptr_vector<Behaviors::Fullback> defenders;
	//Behaviors::Fullback *fullback;

	//BOOST_FOREACH(Robot r, nonPassingRobots){

	//	fullback = new Behaviors::Fullback();
	//	fullback->assignOne(&r);
	//	defenders.push_back(fullback);
	//}

	_passState = Optimizing;

	_passState = Executing;

	_passState = Done;
}

bool Gameplay::Plays::TestPassPlay::run(){
	return true;
}

void Gameplay::Plays::TestPassPlay::initializePlan(){
	initialPlans.clear();
	const Geometry2d::Point ballPos = ball().pos;
	Geometry2d::Point goalBallPos = Geometry2d::Point(-Constants::Field::Width / 2.0, Constants::Field::Length);
	BOOST_FOREACH(Robot *r1, _robots){
		// initialize configurations of length 1
		PassConfig* passConfig = new PassConfig(); // add starting ball position
		passConfig->addPassState(new PassState(&ballPos,PassState::INITIAL)); // add robot with ball
		passConfig->addPassState(new PassState(&ballPos, r1)); // add ending (goal) ball position
		passConfig->addPassState(new PassState(&goalBallPos,PassState::GOAL));
		initialPlans.push_back(passConfig);

		// initialize configurations of length 2
		BOOST_FOREACH(Robot *r2, _robots){
			if(r2->id()==r1->id()){continue;} // don't pass to self
			PassConfig* passConfig = new PassConfig(); // add starting ball position
			passConfig->addPassState(new PassState(&ballPos,PassState::INITIAL)); // add robot1 with ball
			passConfig->addPassState(new PassState(&ballPos, r1)); // add robot2 with ball
			passConfig->addPassState(new PassState(&r2->pos(), r2)); // add ending (goal) ball position
			passConfig->addPassState(new PassState(&goalBallPos,PassState::GOAL));
			initialPlans.push_back(passConfig);

			/*
			// initialize configurations of length 3
			BOOST_FOREACH(Robot *r3, _robots){
				if(r3->id()==r2->id()){continue;} // don't pass to self
												  // note: r1 -> r2 -> r1 is ok.
				PassConfig* passConfig = new PassConfig(); // add starting ball position
				passConfig->addPassState(new PassState(&ballPos,PassState::INITIAL)); // add robot1 with ball
				passConfig->addPassState(new PassState(&ballPos, r1)); // add robot2 with ball
				passConfig->addPassState(new PassState(&r2->pos(), r2)); // add robot3 with ball
				passConfig->addPassState(new PassState(&r3->pos(), r3)); // add ending (goal) ball position
				passConfig->addPassState(new PassState(&goalBallPos,PassState::GOAL));
				initialPlans.push_back(passConfig);
			}
			*/
		}
	}

	//
	// Weight configs
	//
	PassState *prevState = (PassState*)NULL, *thisState, *nextState;
	float ballTravelDist, robotTravelDist, robotRotateDist;
	double ballTravelTime, robotTravelTime, robotRotateTime;
	Point inVector, outVector;

	for(int i=0; i<(int)initialPlans.size(); i++){
		// calculate the timestamps at each state in the config
		//
		for(int j=0; j<initialPlans[i].length(); j++){
			thisState = initialPlans[i].getPassState(j);
			nextState = (j+1<initialPlans[i].length() ? initialPlans[i].getPassState(j+1) : thisState);
			switch(thisState->stateType){
				case PassState::INITIAL :
					thisState->timeEnterState = 0.0;
					thisState->timeLeaveState = 0.0;
					break;
				case PassState::INTERMEDIATE :
					if(prevState->stateType == PassState::INITIAL){
						robotTravelDist = thisState->robot->pos().distTo(thisState->ballPos);
						robotTravelTime = robotTravelDist / APPROXROBOTVELTRANS;
						thisState->timeEnterState = prevState->timeLeaveState + robotTravelTime;
					}else{
						ballTravelDist = thisState->ballPos.distTo(prevState->ballPos);
						ballTravelTime = ballTravelDist / APPROXBALLVEL;
						thisState->timeEnterState = prevState->timeLeaveState + ballTravelTime;
					}
					inVector = prevState->ballPos - thisState->ballPos;
					outVector = nextState->ballPos - thisState->ballPos;
					robotRotateDist = inVector.angleTo(outVector);
					robotRotateTime = robotRotateDist / APPROXROBOTVELROT;
					thisState->timeLeaveState = thisState->timeEnterState + robotRotateTime;
					break;
				case PassState::GOAL :
					ballTravelDist = thisState->ballPos.distTo(prevState->ballPos);
					ballTravelTime = ballTravelDist / APPROXBALLVEL;
					thisState->timeEnterState = prevState->timeLeaveState + ballTravelTime;
					thisState->timeLeaveState = thisState->timeEnterState;
					break;
			}
			prevState = thisState;
		}

		// calculate the total number of opponents that can touch the ball along the path
		//
		int numInteractions = 0;
		for(int j=0; j<initialPlans[i].length(); j++){
			thisState = initialPlans[i].getPassState(j);
			BOOST_FOREACH(Robot *opponentR, gameplay()->opp){
				robotTravelDist = opponentR->pos().distTo(thisState->ballPos);
				robotTravelTime = robotTravelDist / APPROXROBOTVELTRANS;
				if(robotTravelTime < thisState->timeLeaveState){
					numInteractions++;
				}
			}
		}

		initialPlans[i].setWeight(numInteractions);
	}
	initialPlans.sort();

	// print out
	for(int i=0; i<(int)initialPlans.size(); i++){
		cout << "passConfig(" << i << "): " << initialPlans[i] << endl;
	}
}
