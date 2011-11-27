//TODO: fix pass done

#include "PassPlay.hpp"

#include <algorithm>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::PassPlay, "Demos")


namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(PassPlay)
	}
}

ConfigBool *Gameplay::Plays::PassPlay::_use_chipper;
ConfigBool *Gameplay::Plays::PassPlay::_use_line;
ConfigInt *Gameplay::Plays::PassPlay::_kick_power;

Gameplay::Plays::PassPlay::PassPlay(GameplayModule *gameplay):
	Play(gameplay),
	_pass(gameplay)
{
		_passer1HasBall = true;
		positiveX = ball().pos.x > 0;
}

void Gameplay::Plays::PassPlay::createConfiguration(Configuration* cfg)
{
	_use_line  = new ConfigBool(cfg, "PassPlay/Line Shot", true);
	_use_chipper  = new ConfigBool(cfg, "PassPlay/Chipping", false);
	_kick_power = new ConfigInt(cfg, "PassPlay/Kick Power", 50);
}

bool Gameplay::Plays::PassPlay::run()
{
	set<OurRobot *> playRobots = _gameplay->playRobots();
	if (playRobots.size() < 2)
	{
		// No robots
		return false;
	}
	
	if(!_pass.robot1 || playRobots.find(_pass.robot1) == playRobots.end())
	{
		assignNearest(_pass.robot1,playRobots,ball().pos);

	}

	if(!_pass.robot2 || playRobots.find(_pass.robot2) == playRobots.end())
	{
		assignNearest(_pass.robot2,playRobots,ball().pos);
	}
	
	_pass.passPower = *_kick_power;
	_pass.passUseChip = *_use_chipper;
	_pass.passUseLine = *_use_line;


	if(ball().pos.x > 0)
	{
		if(!positiveX)
		{
			OurRobot *temp = _pass.robot1;
			_pass.robot1 = _pass.robot2;
			_pass.robot2 = temp;
			_pass.reset();
		}

		if(_pass.done())
		{
			_pass.reset();
		}

		_pass.setEnable(_pass.robot2->pos.distTo(Geometry2d::Point(-Field_Width / 5, Field_Length / 5)) < 0.5);
		_pass.robot2->move(Geometry2d::Point(-Field_Width / 5, Field_Length / 5),true);
		positiveX = true;
	}
	else
	{
		if(positiveX)
		{
			OurRobot *temp = _pass.robot1;
			_pass.robot1 = _pass.robot2;
			_pass.robot2 = temp;
			_pass.reset();
		}

		if(_pass.done())
		{
			_pass.reset();
		}

		_pass.setEnable(_pass.robot2->pos.distTo(Geometry2d::Point(Field_Width / 5, Field_Length / 5)) < 0.5);
		_pass.robot2->move(Geometry2d::Point(Field_Width / 5, Field_Length / 5),true);
		positiveX = false;
	}
	_pass.run();


	return true;
}
