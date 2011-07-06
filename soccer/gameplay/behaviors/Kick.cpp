#include "Kick.hpp"

#include <stdio.h>
#include <algorithm>

using namespace std;
using namespace Geometry2d;

namespace Gameplay
{
namespace Behaviors
{

Kick::Kick(GameplayModule *gameplay)
	: SingleRobotBehavior(gameplay),
	enableGoalLineShot(false),
	enableLeftDownfieldShot(false),
	enableRightDownfieldShot(false),
	enablePushing(false),
	_pivotKick(gameplay),
	_lineKick(gameplay)
{
	restart();
	setTargetGoal();
}

void Kick::restart() {
	_state = State_PivotKick;
	use_chipper = false;
	dribbler_speed = 127;
	kick_power = 255;
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

bool Kick::findShot(const Geometry2d::Segment& segment, Geometry2d::Segment& result, float min_segment_length) const {
	// calculate available target segments
	WindowEvaluator evaluator(state());
	evaluator.enable_chip = use_chipper;
	evaluator.chip_min_range = minChipRange;
	evaluator.chip_max_range = maxChipRange;
	evaluator.run(ball().pos, segment);
	Window *window = evaluator.best;

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
	Geometry2d::Segment available_target;
	if (!(findShot(_target, available_target, 0.1) 				/// try target first
			  || (enableGoalLineShot && findShot(goal_line, available_target, 0.5)) 		/// try anywhere on goal line
			  || (enableLeftDownfieldShot && findShot(left_downfield, available_target, 0.5))  /// shot off edge of field
			  || (enableRightDownfieldShot && findShot(right_downfield, available_target, 0.5))
			  ))
	{
		robot->addText(QString("Kick:no target"));
		badshot = true;
		available_target = _target;   /// if no other option, try kicking anyway
	}

	const Geometry2d::Point& rPos = robot->pos;
	const float close_thresh = Robot_Radius + Ball_Radius + 0.10;

	// disable obstacle avoidance if we are close to the ball
	if (enablePushing && rPos.nearPoint(ball().pos, close_thresh))
		robot->avoidOpponents(false);
	else
		robot->avoidOpponents(true);

	// there are robots in the way and we are close, disable opponent obstacle avoidance
	if (enablePushing && badshot && rPos.nearPoint(ball().pos, close_thresh)) {
		robot->addText(QString("PushingOpponent"));
		Geometry2d::Point push_goal = rPos + (_target.center() - rPos).normalized() * 2.0;

		// drive forward through robots
		robot->move(push_goal);
		robot->face(_target.center());
		return true;

	} else {
		// use pivot only for now
		_pivotKick.target = available_target;
		_pivotKick.dribble_speed = dribbler_speed;
		_pivotKick.use_chipper = use_chipper;
		_pivotKick.kick_power = kick_power;
		return _pivotKick.run();
	}
}

} // \namespace Behaviors
} // \namespace Gameplay
