/*
 * Passer.cpp
 *
 *  Created on: Nov 8, 2009
 *      Author: alexgc
 */

#include "Passer.hpp"

using namespace Gameplay::Behaviors;

Passer::Passer(GameplayModule *gameplay) :
	Behavior(gameplay)
	{

}

bool Passer::run() {

}

bool Passer::done() {
	return true;
}

float Passer::score(Robot* robot) {
	return 0.0;
}
