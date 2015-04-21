#include "Kick.hpp"

#include <stdio.h>
#include <algorithm>
#include <Utils.hpp>

using namespace std;
using namespace Geometry2d;

namespace Gameplay {
namespace Behaviors {
REGISTER_CONFIGURABLE(Kick)
}
}

ConfigDouble *Gameplay::Behaviors::Kick::_chip_min_range;
ConfigDouble *Gameplay::Behaviors::Kick::_chip_max_range;
ConfigInt *Gameplay::Behaviors::Kick::_chip_min_power;
ConfigInt *Gameplay::Behaviors::Kick::_chip_max_power;

namespace Gameplay
{
namespace Behaviors
{

void Gameplay::Behaviors::Kick::createConfiguration(Configuration *cfg)
{
	_chip_min_range = new ConfigDouble(cfg, "Kick/Chip Min Range", 0.3);
	_chip_max_range	= new ConfigDouble(cfg, "Kick/Chip Max Range", 4.0);
	_chip_min_power = new ConfigInt(cfg, "Kick/Chip Min Power", 100);
	_chip_max_power = new ConfigInt(cfg, "Kick/Chip Max Power", 255);
}

Kick::Kick(GameplayModule *gameplay)
	: SingleRobotBehavior(gameplay),
	enableGoalLineShot(false),
	enableLeftDownfieldShot(false),
	enableRightDownfieldShot(false),
	enablePushing(false),
	take_open_kicks(true),
	_pivotKick(gameplay),
	_lineKick(gameplay),
	lucky(gameplay)
{
	restart();
	setTargetGoal();
}

void Kick::restart() {
	// chipping parameters - should get calculated somehow
	_done = false;
	use_chipper = false;
	use_line_kick = false;
	override_aim = false;
	forceChip = false;
	enableKick = true;
	dribbler_speed = 127;
	kick_power = 255;
	chip_power = 255;
	minChipRange = 0.3;
	maxChipRange = 0.5;
	_pivotKick.restart();
	_lineKick.restart();
}

void Kick::setTargetGoal() {
	_target.pt[0] = Point(Field_GoalWidth / 2, Field_Length);
	_target.pt[1] = Point(-Field_GoalWidth / 2, Field_Length);
	_pivotKick.target = _target;
}

void Kick::setTarget(const Geometry2d::Segment &seg) {
	_target = seg;
	_pivotKick.target = _target;
}

void Kick::calculateChipPower(double dist)
{
	double range_area = *_chip_max_range - *_chip_min_range;
	double ratio = (dist - *_chip_min_range)/range_area;
	uint8_t power_area = *_chip_max_power - *_chip_min_power;
	double power = ratio * (double) power_area + *_chip_min_power;
	chip_power = (uint8_t) clamp((double) power, (double) _chip_min_power->value(), 255.0);
}

bool Kick::findShot(const Geometry2d::Segment& segment, Geometry2d::Segment& result, bool chip, float min_segment_length) const {
	// calculate available target segments
	WindowEvaluator evaluator(state());
	evaluator.enable_chip = chip;
	evaluator.chip_min_range = minChipRange;
	evaluator.chip_max_range = maxChipRange;
	evaluator.run(ball().pos, segment);
	Window *window = evaluator.best();

	// check for output
	if(window && window->segment.length() > min_segment_length)	{
		result = window->segment;
		return true;
	} else {
		return false;
	}
}

bool Kick::run() {
	if (!robot || !robot->visible)
	{
		return false;
	}

	// assign robots - assume updated from driver play
	_pivotKick.robot = robot;
	_lineKick.robot = robot;

	const Segment goal_line(Point(Field_Width / 2, Field_Length), Point(-Field_Width / 2, Field_Length));
	const Segment left_downfield(Point(-Field_Width / 2, Field_Length * 0.8), Point(-Field_Width / 2, Field_Length));
	const Segment right_downfield(Point(Field_Width / 2, Field_Length * 0.8), Point(Field_Width / 2, Field_Length));

	// check for shot on goal
	bool badshot = false;
	Geometry2d::Segment available_target = _target;
	bool must_use_chip = false;
	if (!override_aim)
	{
		if (!forceChip && !(findShot(_target, available_target, false, 0.1) 				/// try target first with kicker
				|| (enableGoalLineShot && findShot(goal_line, available_target, false, 0.5)) 		/// try anywhere on goal line
				|| (enableLeftDownfieldShot && findShot(left_downfield, available_target, false, 0.5))  /// shot off edge of field
				|| (enableRightDownfieldShot && findShot(right_downfield, available_target, false, 0.5))
		))
		{
			robot->addText(QString("Kick: no kick target"));
			badshot = true;
			available_target = _target;   /// if no other option, try kicking anyway
		} else if (use_chipper && (findShot(_target, available_target, true, 0.1)  				/// try target with chipper
				|| (enableGoalLineShot && findShot(goal_line, available_target, true, 0.5)) 		/// try anywhere on goal line
				|| (enableLeftDownfieldShot && findShot(left_downfield, available_target, true, 0.5))  /// shot off edge of field
				|| (enableRightDownfieldShot && findShot(right_downfield, available_target, true, 0.5))
		))
		{
			robot->addText(QString("Kick:chipping"));
			available_target = _target;   /// if no other option, try kicking anyway
			must_use_chip = true;
		} else {
			badshot = true;
		}
	}

	const Geometry2d::Point& rPos = robot->pos;
	const float close_thresh = Robot_Radius + Ball_Radius + 0.10;

	// disable obstacle avoidance if we are close to the ball
	if (enablePushing && rPos.nearPoint(ball().pos, close_thresh))
		robot->avoidOpponents(false);
	else
		robot->avoidOpponents(true);

	bool result = true;
	// there are robots in the way and we are close, disable opponent obstacle avoidance
	if (enablePushing && badshot && rPos.nearPoint(ball().pos, close_thresh)) {
		robot->addText(QString("PushingOpponent"));
		Geometry2d::Point push_goal = rPos + (_target.center() - rPos).normalized() * 2.0;

		// drive forward through robots
		robot->move(push_goal);
		robot->face(_target.center());
		result = true;

	} else if (use_line_kick)
	{
		_lineKick.target = available_target.center();
		_lineKick.use_chipper = must_use_chip;
		_lineKick.kick_power = (must_use_chip) ? chip_power : kick_power;
		_lineKick.enable_kick = enableKick;
		kickReady = _lineKick.kick_ready;
		result = _lineKick.run();
		if (_lineKick.done())
		{
			_done = true;
		}
	} else
	{
		_pivotKick.target = available_target;
		_pivotKick.dribble_speed = dribbler_speed;
		_pivotKick.use_chipper = use_chipper;
		_pivotKick.kick_power = (use_chipper) ? chip_power : kick_power;
		_pivotKick.enable_kick = enableKick;
		result = _pivotKick.run();
		if (_pivotKick.done())
		{
			_done = true;
		}
	}
	// Take opportunities!
	if(take_open_kicks) {
		Luck_Options opt = lucky.get_options();
		bool ok = lucky.use_kick(robot, opt);
		ok &= lucky.open_kick(robot, opt); // Checks if E open shot to goal
		if(ok) {
			robot->addText("Taking Opportunity!", Qt::magenta, "Kick");
			robot->kick(255);

			// We kicked, therefore exit
			result = false;
		}
	}

	return result;
}

} // \namespace Behaviors
} // \namespace Gameplay
