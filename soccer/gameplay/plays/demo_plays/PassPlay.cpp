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
}

void Gameplay::Plays::PassPlay::createConfiguration(Configuration* cfg)
{
	_use_line  = new ConfigBool(cfg, "PassPlay/Line Shot", true);
	_use_chipper  = new ConfigBool(cfg, "PassPlay/Chipping", false);
	_kick_power = new ConfigInt(cfg, "PassPlay/Kick Power", 127);
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

	if(_passer1HasBall && _pass.done())
	{

		OurRobot *temp = _pass.robot1;
		_pass.robot1 = _pass.robot2;
		_pass.robot2 = temp;

		_pass.reset();
		_passer1HasBall = false;

	}
	else if( !_passer1HasBall && _pass.done())
	{

		OurRobot *temp = _pass.robot1;
		_pass.robot1 = _pass.robot2;
		_pass.robot2 = temp;
		_pass.reset();
		_passer1HasBall = true;

	}
	_pass.run();

	return true;
}
