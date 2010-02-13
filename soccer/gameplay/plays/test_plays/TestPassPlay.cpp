/*
 * TestPassPlay.cpp
 *
 *  Created on: Nov 29th
 *      Author: Philip Rogers
 *      Author:
 */

#include <QColor>
#include "TestPassPlay.hpp"

#define TIMEMARGIN 3.5 // seconds a play can deviate from plan before abort
#define ROBOTSUCCESSMARGIN 0.05 // if robot within this region, a move is complete
#define ROBOTKICKSUCCESSMARGIN 0.05 // if kicking robot within this region, proceed to kick
#define BALLSUCCESSMARGIN 0.2 // if robot within this region, a move is complete

using namespace Geometry2d;
using namespace std;

Gameplay::Plays::TestPassPlay::TestPassPlay(GameplayModule *gameplay)
: Play(gameplay), passPlanner_(gameplay) {

}

void Gameplay::Plays::TestPassPlay::assign(set<Robot *> &available){
	passPlanner_.assign(available);
}

bool Gameplay::Plays::TestPassPlay::run(){
	passPlanner_.run();

	return true;
}
