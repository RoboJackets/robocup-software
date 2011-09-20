#include "PassShootPlay.hpp"

#include <algorithm>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::PassShootPlay, "Demos")

static bool shellLessThan(Robot *r1, Robot *r2)
{
	return r1->shell() < r2->shell();
}

Gameplay::Plays::PassShootPlay::PassShootPlay(GameplayModule *gameplay):
	Play(gameplay),
	_passer(gameplay),
	_kicker(gameplay),
	_passerHasBall(true)
{
}

bool Gameplay::Plays::PassShootPlay::run()
{
	set<OurRobot *> playRobots = _gameplay->playRobots();
	if (playRobots.size() < 2)
	{
		// No robots
		return false;
	}
	
	if(!_passer.robot || playRobots.find(_passer.robot) == playRobots.end())
	{
		assignNearest(_passer.robot,playRobots,ball().pos);

	}

	if(!_kicker.robot || playRobots.find(_kicker.robot) == playRobots.end())
	{
		assignNearest(_kicker.robot,playRobots,Geometry2d::Point(0,Field_Length));
	}
	
//	_passer2.robot->move(Geometry2d::Point(0,Field_Length / 2));


	if(_passerHasBall)
	{
		_kicker.robot->face(Geometry2d::Point(0,Field_Length));

		_passer.setTarget(_kicker.robot->kickerBar());
		_passer.kick_power = 100;
		_passer.run();

		if(_passer.done())
		{
			_passer.restart();
			_passerHasBall = false;
		}
	}
	else
	{
		_kicker.run();

	}
	
	return true;
}
