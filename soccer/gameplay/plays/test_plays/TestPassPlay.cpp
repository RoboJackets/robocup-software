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
	AnalyticPassPlanner::generateAllConfigs(ball().pos,_robots,initialPlans);
	AnalyticPassPlanner::evaluateConfigs(_robots,_gameplay->opp,initialPlans);

	// print out
	for(int i=0; i<(int)initialPlans.size(); i++){
		cout << "passConfig(" << i << "): " << initialPlans[i] << endl;
	}
}
