/*
 * TestOptimizedPassPlay.cpp
 *
 *  Created on: Nov 29th
 *      Author: Philip Rogers
 *      Author:
 */

#include <QColor>
#include "TestOptimizedPassPlay.hpp"

//#define TIMEMARGIN 3.5 // seconds a play can deviate from plan before abort
//#define ROBOTSUCCESSMARGIN 0.05 // if robot within this region, a move is complete
//#define ROBOTKICKSUCCESSMARGIN 0.05 // if kicking robot within this region, proceed to kick
//#define BALLSUCCESSMARGIN 0.2 // if robot within this region, a move is complete

using namespace Geometry2d;
using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::TestOptimizedPassPlay, "Tests")

Gameplay::Plays::TestOptimizedPassPlay::TestOptimizedPassPlay(GameplayModule *gameplay)
: Play(gameplay), passPlanner_(gameplay, true) {

}

bool Gameplay::Plays::TestOptimizedPassPlay::assign(set<Robot *> &available){
	return passPlanner_.assign(available);
}

bool Gameplay::Plays::TestOptimizedPassPlay::run(){
	// force enable the optimizer
	passPlanner_.enableOptimization(true);

	// set margins
	double time_margin = 5.5,
		   robotsuccess_margin = 0.05,
		   ballsuccess_margin = 0.2;
	passPlanner_.setMargins(time_margin, robotsuccess_margin, ballsuccess_margin);

	// execute
	passPlanner_.run();

	return true;
}
