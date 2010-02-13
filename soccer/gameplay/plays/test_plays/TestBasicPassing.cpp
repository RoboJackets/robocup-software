/*
 * TestBasicPassing.cpp
 *
 *  Created on: Nov 8, 2009
 *      Author: alexgc
 */

#include "TestBasicPassing.hpp"

using namespace Geometry2d;
using namespace std;

Gameplay::Plays::TestBasicPassing::TestBasicPassing(GameplayModule *gameplay):
	Play(gameplay),
	_passer(gameplay),
	_receiver(gameplay)
{
	_passState = CreateTrajectory;
}

bool Gameplay::Plays::TestBasicPassing::applicable()
{
	return true;
}

void Gameplay::Plays::TestBasicPassing::assign(set<Robot *> &available)
{
	_passer.assign(available);
	_receiver.assign(available);
	
	_passer.targetRobot = _receiver.robot();
}

bool Gameplay::Plays::TestBasicPassing::run()
{
	if (!_gameplay->state()->gameState.playing())
	{
		_passer.restart();
	}

	_receiver.face = _passer.robot()->pos();
	_receiver.target = _receiver.robot()->pos();

	_passer.run();
	_receiver.run();
	
	return true;
}
