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
ConfigInt *Gameplay::Plays::PassPlay::_backup_time;
ConfigDouble *Gameplay::Plays::PassPlay::_backup_speed;

static bool shellLessThan(Robot *r1, Robot *r2)
{
	return r1->shell() < r2->shell();
}

Gameplay::Plays::PassPlay::PassPlay(GameplayModule *gameplay):
	Play(gameplay),
	_passer1(gameplay),
	_passer2(gameplay)
{
		readyTime = 0;
		_passer1HasBall = true;
}

void Gameplay::Plays::PassPlay::createConfiguration(Configuration* cfg)
{
	_use_line  = new ConfigBool(cfg, "PassPlay/Line Shot", true);
	_use_chipper  = new ConfigBool(cfg, "PassPlay/Chipping", false);
	_kick_power = new ConfigInt(cfg, "PassPlay/Kick Power", 127);
	_backup_time = new ConfigInt(cfg, "PassPlay/Backup Time", 1000);
	_backup_speed = new ConfigDouble(cfg, "PassPlay/Backup Speed", 0.05);
}

bool Gameplay::Plays::PassPlay::run()
{
	set<OurRobot *> playRobots = _gameplay->playRobots();
	if (playRobots.size() < 2)
	{
		// No robots
		return false;
	}
	
	if(!_passer1.robot || playRobots.find(_passer1.robot) == playRobots.end())
	{
		assignNearest(_passer1.robot,playRobots,ball().pos);

	}

	if(!_passer2.robot || playRobots.find(_passer2.robot) == playRobots.end())
	{
		assignNearest(_passer2.robot,playRobots,ball().pos);
	}
	
//	_passer2.robot->move(Geometry2d::Point(0,Field_Length / 2));


	if(_passer1HasBall /*&& (ball().pos.distTo(_passer1.robot->pos) < 0.5 || ball().vel.mag() < 0.1)*/)
	{
		_passer2.robot->face(_passer1.robot->pos);

		_passer1.setTarget(_passer2.robot->kickerBar());
		_passer1.kick_power = *_kick_power;
		_passer1.use_line_kick = *_use_line;
		_passer1.use_chipper = *_use_chipper;
		_passer1.enableKick = false;
		if(_passer1.kickReady)
		{
			_passer2.robot->bodyVelocity(Geometry2d::Point(-(*_backup_speed),0));
			_passer2.robot->angularVelocity(0);
			if(readyTime == 0)
			{
				readyTime = timestamp();
			}
			else if((timestamp() - readyTime) > *_backup_time)
			{
				_passer1.enableKick = true;
			}
		}

		_passer1.run();

		if(_passer1.done())
		{
			readyTime = 0;
			_passer1.restart();
			_passer1HasBall = false;
		}
	}
	else if( !_passer1HasBall /*&& (ball().pos.distTo(_passer2.robot->pos) < 0.5 || ball().vel.mag() < 0.1)*/)
	{
		_passer1.robot->face(_passer2.robot->pos);
		_passer2.setTarget(_passer1.robot->kickerBar());
		_passer2.kick_power = *_kick_power;
		_passer2.use_line_kick = *_use_line;
		_passer2.use_chipper = *_use_chipper;
		_passer2.enableKick = false;

		if(_passer2.kickReady)
		{
			_passer1.robot->bodyVelocity(Geometry2d::Point(-(*_backup_speed),0));
			_passer1.robot->angularVelocity(0);
			if(readyTime == 0)
			{
				readyTime = timestamp();
			}
			else if((timestamp() - readyTime) > *_backup_time)
			{
				_passer2.enableKick = true;
			}
		}

		_passer2.run();
		if(_passer2.done())
		{
			readyTime = 0;
			_passer2.restart();
			_passer1HasBall = true;
		}
	}
	
	return true;
}
