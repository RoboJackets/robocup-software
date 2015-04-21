#include <limits>

#include "EmptyGoal.hpp"

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::EmptyGoal, "Playing")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(EmptyGoal)
	}
}

ConfigDouble *Gameplay::Plays::EmptyGoal::_scale_speed;
ConfigDouble *Gameplay::Plays::EmptyGoal::_scale_acc;
ConfigDouble *Gameplay::Plays::EmptyGoal::_scale_w;

void Gameplay::Plays::EmptyGoal::createConfiguration(Configuration *cfg)
{
	_scale_speed = new ConfigDouble(cfg, "EmptyGoal/Scale Speed", 0.5);
	_scale_acc = new ConfigDouble(cfg, "EmptyGoal/Scale Accel", 0.5);
	_scale_w = new ConfigDouble(cfg, "EmptyGoal/Scale W", 0.3);
}

Gameplay::Plays::EmptyGoal::EmptyGoal(GameplayModule *gameplay):
Play(gameplay),
_leftDefender(gameplay, Behaviors::Defender::Left),
_rightDefender(gameplay, Behaviors::Defender::Right),
_striker(gameplay)
{
	_leftDefender.otherDefenders.insert(&_rightDefender);
	_rightDefender.otherDefenders.insert(&_leftDefender);
}

float Gameplay::Plays::EmptyGoal::score ( Gameplay::GameplayModule* gameplay )
{
	// only run if we are playing and not in a restart
	bool refApplicable = gameplay->state()->gameState.playing();
	return refApplicable ? 0 : INFINITY;
}

bool Gameplay::Plays::EmptyGoal::run()
{
	// handle assignments
	set<OurRobot *> available = _gameplay->playRobots();

	// project the ball forward (dampened)
	const float proj_time = 0.75; // seconds to project
	const float dampen_factor = 0.9; // accounts for slowing over time
	Geometry2d::Point ballProj = ball().pos + ball().vel * proj_time * dampen_factor;

	// Get a striker first to ensure there a robot that can kick
	assignNearestKicker(_striker.robot, available, ballProj);

	// defense first - get closest to goal to choose sides properly
	assignNearest(_leftDefender.robot, available, Geometry2d::Point(-Field_GoalWidth/2, 0.0));
	assignNearest(_rightDefender.robot, available, Geometry2d::Point(Field_GoalWidth/2, 0.0));

	if (!available.empty())
	{
		OurRobot * idle = *available.begin();
		idle->move(Point(Field_Width / 2.0, 0.0));
	}

	// manually reset any kickers so they keep kicking
	if (_striker.done())
		_striker.restart();

	// set flags for inner behaviors
	_striker.use_line_kick = true;
	_striker.setTargetGoal();
	_striker.use_chipper = false;
	
	// execute behaviors
	if (_striker.robot) _striker.run();
	if (_leftDefender.robot) _leftDefender.run();
	if (_rightDefender.robot) _rightDefender.run();

	return true;
}
