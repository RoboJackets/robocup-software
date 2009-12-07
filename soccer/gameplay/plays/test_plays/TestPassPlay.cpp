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

	/* THIS SEG FAULTS

	PassConfig bestPassConfig = initialPlans.front();	// just pick up the first plan (temporary!)
	boost::ptr_vector<Robot> nonPassingRobots;	// vector of robots which are not part of the
												// selected passing strategy
	BOOST_FOREACH(Robot *r, _robots){		// initialize the nonPassingRobots with all available
		nonPassingRobots.push_back(r);		// robots
	}
	BOOST_FOREACH(PassState passState, bestPassConfig.passStateVector){
		Robot *r = passState.controllingRobot;	// robot which takes part in the current state
		if (r != 0){		// checking if the pointer is null
			int id = r->id();
			boost::ptr_vector<Robot>::iterator it;
			// removing the passing robot from the vector of nonPassingRobots
			for (it = nonPassingRobots.begin(); it != nonPassingRobots.end(); ++it){
				if (it->id() == id){
					nonPassingRobots.erase(it);
					break;
				}
			}
		}
	}
*/
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
	PassConfig* passConfig;
	BOOST_FOREACH(Robot *r1, _robots){
		// initialize configurations of length 1
		passConfig = new PassConfig(); // add starting ball position
		passConfig->addPassState(new PassState(&ballPos)); // add robot with ball
		passConfig->addPassState(new PassState(&ballPos, r1)); // add ending (goal) ball position
		passConfig->addPassState(new PassState(&goalBallPos));
		initialPlans.push_back(passConfig);

		// initialize configurations of length 2
		BOOST_FOREACH(Robot *r2, _robots){
			if(r2->id()==r1->id()){continue;} // don't pass to self
			passConfig = new PassConfig(); // add starting ball position
			passConfig->addPassState(new PassState(&ballPos)); // add robot1 with ball
			passConfig->addPassState(new PassState(&ballPos, r1)); // add robot2 with ball
			passConfig->addPassState(new PassState(&r2->pos(), r2)); // add ending (goal) ball position
			passConfig->addPassState(new PassState(&goalBallPos));
			initialPlans.push_back(passConfig);

			// initialize configurations of length 3
			BOOST_FOREACH(Robot *r3, _robots){
				if(r3->id()==r2->id()){continue;} // don't pass to self
												  // note: r1 -> r2 -> r1 is ok.
				passConfig = new PassConfig(); // add starting ball position
				passConfig->addPassState(new PassState(&ballPos)); // add robot1 with ball
				passConfig->addPassState(new PassState(&ballPos, r1)); // add robot2 with ball
				passConfig->addPassState(new PassState(&r2->pos(), r2)); // add robot3 with ball
				passConfig->addPassState(new PassState(&r3->pos(), r3)); // add ending (goal) ball position
				passConfig->addPassState(new PassState(&goalBallPos));
				initialPlans.push_back(passConfig);
			}
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
		//cout << "number of states in config [" << i << "]: " << initialPlans.size() << ", weight: " << passConfig.weight << endl;
		cout << "passConfig(" << i << "): " << passConfig << endl;
	}
}
