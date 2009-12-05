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
	// assign all the visible, available robots to _robots
	BOOST_FOREACH(Robot *r, available){
		if(r->visible()){
			_robots.insert(r);
			available.erase(r);
		}
	}

	// This stuff does not belong here, but for this test, we'll keep it here
	// b/c we need to assign robots based on the # of bots used in final plan

	_passState = Initializing;
	initializePlan();

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
		//
		// initialize configurations of length 1
		//
		PassConfig* passConfig = new PassConfig();
		// add starting ball position
		passConfig->addPassState(new PassState(&ballPos));
		// add robot with ball
		passConfig->addPassState(new PassState(&ballPos, r1));
		// add ending (goal) ball position
		passConfig->addPassState(new PassState(&goalBallPos));
		initialPlans.push_back(passConfig);

		//
		// initialize configurations of length 2
		//
		BOOST_FOREACH(Robot *r2, _robots){
			if(r2->id()==r1->id()){continue;} // don't pass to self
			PassConfig* passConfig = new PassConfig();
			// add starting ball position
			passConfig->addPassState(new PassState(&ballPos));
			// add robot1 with ball
			passConfig->addPassState(new PassState(&ballPos, r1));
			// add robot2 with ball
			passConfig->addPassState(new PassState(&r2->pos(), r2));
			// add ending (goal) ball position
			passConfig->addPassState(new PassState(&goalBallPos));
			initialPlans.push_back(passConfig);
		}
	}

	//
	// Weight configs
	//
	cout << "number of configs:" << initialPlans.size() << endl;
	for(int i=0; i<(int)initialPlans.size(); i++){
		PassConfig passConfig = initialPlans.at(i);
		// todo
		passConfig.setWeight(1.0);
		cout << "number of states in config [" << i << "]: " << initialPlans.size() << ", weight: " << passConfig.weight << endl;
	}

}
