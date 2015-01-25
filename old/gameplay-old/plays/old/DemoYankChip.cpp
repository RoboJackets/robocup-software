#include "DemoYankChip.hpp"


using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoYankChip, "Demos")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(DemoYankChip)
	}
}

ConfigInt *Gameplay::Plays::DemoYankChip::_dribblerSpeed;
ConfigDouble *Gameplay::Plays::DemoYankChip::_backupDist;
ConfigBool *Gameplay::Plays::DemoYankChip::_useTarget;

void Gameplay::Plays::DemoYankChip::createConfiguration(Configuration* cfg)
{
	_dribblerSpeed = new ConfigInt(cfg, "DemoYankChip/Dribber Speed", 127);
	_backupDist  = new ConfigDouble(cfg, "DemoYankChip/Backup Distance", 0.4);
	_useTarget  = new ConfigBool(cfg, "DemoYankChip/Enable Aiming", true);
}

Gameplay::Plays::DemoYankChip::DemoYankChip(GameplayModule *gameplay):
Play(gameplay),
_yank(gameplay)
{
}

bool Gameplay::Plays::DemoYankChip::run()
{
	Geometry2d::Point ballPos = ball().pos;

	set<OurRobot *> available = _gameplay->playRobots();
	assignNearest(_yank.robot, available, ballPos);

	_yank.target = Geometry2d::Point(0, Field_Length);

	// if we have kicked, we want to reset
	if (_yank.done() &&  ball().valid &&
			(!ballPos.nearPoint(Geometry2d::Point(0, Field_Length), Field_ArcRadius)))
	{
		_yank.restart();
	}

	// set flags from parameters
	_yank.dribble_speed = *_dribblerSpeed;
	_yank.backup_distance = *_backupDist;

	_yank.run();

	return true;
}
