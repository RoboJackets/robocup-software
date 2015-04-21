#include "PassShootPlay.hpp"

#include <algorithm>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::PassShootPlay, "Demos")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(PassShootPlay)
	}
}

ConfigBool *Gameplay::Plays::PassShootPlay::_shoot_use_chipper;
ConfigBool *Gameplay::Plays::PassShootPlay::_shoot_use_line;
ConfigBool *Gameplay::Plays::PassShootPlay::_pass_use_chipper;
ConfigBool *Gameplay::Plays::PassShootPlay::_pass_use_line;
ConfigInt  *Gameplay::Plays::PassShootPlay::_pass_kick_power;
ConfigInt  *Gameplay::Plays::PassShootPlay::_shoot_kick_power;



Gameplay::Plays::PassShootPlay::PassShootPlay(GameplayModule *gameplay):
	Play(gameplay),
	_passer(gameplay),
	_kicker(gameplay),
	_passerHasBall(true)
{
}

void Gameplay::Plays::PassShootPlay::createConfiguration(Configuration* cfg)
{
	_shoot_use_chipper  = new ConfigBool(cfg, "PassShootPlay/Shooter Chipping", false);
	_shoot_use_line  = new ConfigBool(cfg, "PassShootPlay/Shooter Line Shot", false);
	_shoot_kick_power = new ConfigInt(cfg, "PassShootPlay/Shooter Power", 127);
	_pass_use_chipper  = new ConfigBool(cfg, "PassShootPlay/Passer Chipping", false);
	_pass_use_line  = new ConfigBool(cfg, "PassShootPlay/Passer Line Shot", false);
	_pass_kick_power = new ConfigInt(cfg, "PassShootPlay/Passer Power", 127);
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
		_kicker.robot->face(Geometry2d::Point(0, Field_Length));
		_passer.setTarget(Geometry2d::Segment(_kicker.robot->pos, Geometry2d::Point(0, Field_Length)));
		_passer.kick_power = *_pass_kick_power;
		_passer.use_line_kick = *_pass_use_line;
		_passer.run();

		if(_passer.done())
		{
			_passer.restart();
			_passerHasBall = false;
		}
	}
	else
	{
		if(ball().pos.y > Field_Length)
		{
			_passerHasBall = true;
			_kicker.restart();
		}
		if(_kicker.done())
		{
			_kicker.restart();
		}
		_kicker.kick_power = *_shoot_kick_power;
		_kicker.use_line_kick = *_shoot_use_line;
		_kicker.use_chipper = *_shoot_use_chipper;
		_kicker.run();

	}
	

	return true;
}
