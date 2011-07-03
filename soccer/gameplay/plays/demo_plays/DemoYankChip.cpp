#include "DemoYankChip.hpp"

#include <boost/foreach.hpp>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::DemoYankChip, "Demos")

Gameplay::Plays::DemoYankChip::DemoYankChip(GameplayModule *gameplay):
Play(gameplay),
_yank(gameplay)
{
	_dribblerSpeed = config()->createInt("DemoYankChip/Dribber Speed", 127);
	_backupDist = config()->createDouble("DemoYankChip/Backup Distance", 0.4);
	_useTarget = config()->createBool("DemoYankChip/Enable Aiming", true);
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
