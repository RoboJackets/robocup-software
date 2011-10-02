#include "OurFreekick.hpp"

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::OurFreekick, "Restarts")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(OurFreekick)
	}
}

ConfigBool *Gameplay::Plays::OurFreekick::_enableGoalLineShot;
ConfigBool *Gameplay::Plays::OurFreekick::_enableLeftDownfieldShot;
ConfigBool *Gameplay::Plays::OurFreekick::_enableRightDownfieldShot;
ConfigBool *Gameplay::Plays::OurFreekick::_enableChipper;
ConfigDouble *Gameplay::Plays::OurFreekick::_minChipRange;
ConfigDouble *Gameplay::Plays::OurFreekick::_maxChipRange;

void Gameplay::Plays::OurFreekick::createConfiguration(Configuration *cfg)
{
	_enableGoalLineShot = new ConfigBool(cfg, "OurFreekick/Enable GoalLine Shot", true);
	_enableLeftDownfieldShot = new ConfigBool(cfg, "OurFreekick/Enable Left Downfield Shot", true);
	_enableRightDownfieldShot = new ConfigBool(cfg, "OurFreekick/Enable Right Downfield Shot", true);
	_enableChipper = new ConfigBool(cfg, "OurFreekick/Enable Chipper", true);
	_minChipRange = new ConfigDouble(cfg, "OurFreekick/Min Chip Range", 0.3);
	_maxChipRange = new ConfigDouble(cfg, "OurFreekick/Max Chip Range", 2.0);
}

Gameplay::Plays::OurFreekick::OurFreekick(GameplayModule *gameplay):
	Play(gameplay),
	_kicker(gameplay),
	_bump(gameplay),
	_center(gameplay),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_fullback2(gameplay, Behaviors::Fullback::Right),
	_pdt(gameplay, &_kicker)
{
	_center.target = _gameplay->centerMatrix() * Geometry2d::Point(0, 1.5);
	_bump.target = Geometry2d::Point(0.0, Field_Length);

	_fullback2.otherFullbacks.insert(&_fullback1);
	_fullback1.otherFullbacks.insert(&_fullback2);
}

float Gameplay::Plays::OurFreekick::score ( Gameplay::GameplayModule* gameplay )
{
	const GameState &gs = gameplay->state()->gameState;
	return (gs.setupRestart() && gs.ourFreeKick()) ? 10 : INFINITY;
}

bool Gameplay::Plays::OurFreekick::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	
	bool chipper_available = true, kicker_available = true;
	if (!assignNearestChipper(_kicker.robot, available, ball().pos))
	{
		chipper_available = false;
		if (!assignNearestKicker(_kicker.robot, available, ball().pos))
		{
			kicker_available = false;
			assignNearest(_bump.robot, available, ball().pos);
			_pdt.resetBehavior(&_bump);
		}
	}
	_pdt.resetBehavior(&_kicker);

	_kicker.setTargetGoal();
	_kicker.calculateChipPower(ball().pos.distTo(Point(0.0, Field_Length)));

	_bump.target = Point(0.0, Field_Length);
	_bump.run();

	// setup kicker from parameters - want to use chipper when possible
	_kicker.use_line_kick = true;
	_kicker.enableGoalLineShot = *_enableGoalLineShot;
	_kicker.enableLeftDownfieldShot = *_enableLeftDownfieldShot;
	_kicker.enableRightDownfieldShot = *_enableRightDownfieldShot;
	_kicker.use_chipper = chipper_available && *_enableChipper; // always use chipper when enabled
	_kicker.minChipRange = *_minChipRange;
	_kicker.maxChipRange = *_maxChipRange;

	_pdt.backoff.robots.clear();
	_pdt.backoff.robots.insert(_kicker.robot);
	assignNearest(_center.robot, available, _center.target);
	assignNearest(_fullback1.robot, available, Geometry2d::Point(-Field_GoalHeight/2.0, 0.0));
	assignNearest(_fullback2.robot, available, Geometry2d::Point( Field_GoalHeight/2.0, 0.0));
	
	if (!chipper_available && !kicker_available && _bump.robot)
	{
		_pdt.resetBehavior(&_bump);
	} else
	{
		_pdt.resetBehavior(&_kicker);
	}

	_pdt.run();
	_center.run();
	_fullback1.run();
	_fullback2.run();
	
	return _pdt.keepRunning();
}
