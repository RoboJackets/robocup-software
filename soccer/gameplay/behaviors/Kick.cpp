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
	: SingleRobotBehavior(gameplay), _pivotKick(gameplay), _lineKick(gameplay)
{
	_state = State_PivotKick;
	setTargetGoal();
}

void Kick::restart() {
	_state = State_PivotKick;
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
	const Segment left_downfield(Point(-Field_Width / 2, Field_Length * 2.0/3.0), Point(-Field_Width / 2, Field_Length));
	const Segment right_downfield(Point(Field_Width / 2, Field_Length * 2.0/3.0), Point(Field_Width / 2, Field_Length));

	// check for shot on goal
	Geometry2d::Segment available_target;
	if (!(findShot(_target, available_target, 0.04) ||				/// try target first
			  findShot(goal_line, available_target, 0.5) ||				/// try anywhere on goal line
			  findShot(left_downfield, available_target, 0.5) ||  /// shot off edge of field
			  findShot(right_downfield, available_target, 0.5))) {
		robot->addText(QString("Kick:no target"));
		available_target = _target;   /// if no other option, try kicking anyway
	}
	// FIXME: need to try bumping here

	// use pivot only for now
	_pivotKick.target = available_target;
	return _pivotKick.run();
}

} // \namespace Behaviors
} // \namespace Gameplay
