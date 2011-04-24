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

bool Kick::run() {
	if (!robot || !robot->visible)
	{
		return false;
	}

	// calculate available target segments

	// PLACEHOLDER: use just pivot
	return _pivotKick.run();
}

} // \namespace Behaviors
} // \namespace Gameplay
