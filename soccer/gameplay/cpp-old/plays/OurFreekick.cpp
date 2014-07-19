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
ConfigDouble *Gameplay::Plays::OurFreekick::_avoidShot;

void Gameplay::Plays::OurFreekick::createConfiguration(Configuration *cfg)
{
	_enableGoalLineShot = new ConfigBool(cfg, "OurFreekick/Enable GoalLine Shot", true);
	_enableLeftDownfieldShot = new ConfigBool(cfg, "OurFreekick/Enable Left Downfield Shot", true);
	_enableRightDownfieldShot = new ConfigBool(cfg, "OurFreekick/Enable Right Downfield Shot", true);
	_enableChipper = new ConfigBool(cfg, "OurFreekick/Enable Chipper", true);
	_minChipRange = new ConfigDouble(cfg, "OurFreekick/Min Chip Range", 0.3);
	_maxChipRange = new ConfigDouble(cfg, "OurFreekick/Max Chip Range", 2.0);
	_avoidShot = new ConfigDouble(cfg, "OurFreekick/Avoid Shot", 0.3);
}

Gameplay::Plays::OurFreekick::OurFreekick(GameplayModule *gameplay):
	Play(gameplay),
	_kicker(gameplay),
	_bump(gameplay),
	_center1(gameplay),
	_center2(gameplay),
	_defender1(gameplay, Behaviors::Defender::Left),
	_defender2(gameplay, Behaviors::Defender::Right),
	_pdt(gameplay, &_kicker)
{
	_center1.target = _gameplay->centerMatrix() * Geometry2d::Point(0, 1.5);
	_center2.target = _gameplay->centerMatrix() * Geometry2d::Point(1, 1.5);
	_bump.target = Geometry2d::Point(0.0, Field_Length);

	_defender2.otherDefenders.insert(&_defender1);
	_defender1.otherDefenders.insert(&_defender2);
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
	assignNearest(_center1.robot, available, _center1.target);
	assignNearest(_center2.robot, available, _center2.target);
	assignNearest(_defender1.robot, available, Geometry2d::Point(-Field_GoalHeight/2.0, 0.0));
	assignNearest(_defender2.robot, available, Geometry2d::Point( Field_GoalHeight/2.0, 0.0));

	// get out of ways of shots
	if (_kicker.robot && _kicker.robot->pos.nearPoint(ball().pos, *_avoidShot)) {
		Polygon shot_obs;
		shot_obs.vertices.push_back(Geometry2d::Point(Field_GoalWidth / 2, Field_Length));
		shot_obs.vertices.push_back(Geometry2d::Point(-Field_GoalWidth / 2, Field_Length));
		shot_obs.vertices.push_back(ball().pos);
		if(_center1.robot)
			_center1.robot->localObstacles(std::shared_ptr<Obstacle>(new PolygonObstacle(shot_obs)));
		if(_center2.robot)
			_center2.robot->localObstacles(std::shared_ptr<Obstacle>(new PolygonObstacle(shot_obs)));
	}

	if (!chipper_available && !kicker_available && _bump.robot)
	{
		_pdt.resetBehavior(&_bump);
	} else
	{
		_pdt.resetBehavior(&_kicker);
	}

	_pdt.run();
	_center1.run();
	_center2.run();
	_defender1.run();
	_defender2.run();
	
	return _pdt.keepRunning();
}
