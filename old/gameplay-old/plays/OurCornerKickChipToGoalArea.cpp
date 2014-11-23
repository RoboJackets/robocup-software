/*
 * OurCornerKickChipToGoalArea.cpp
 *
 *  Created on: Jun 26, 2013
 *      Author: matt
 */

#include "OurCornerKickChipToGoalArea.hpp"


namespace Gameplay {
namespace Plays {

REGISTER_PLAY_CATEGORY(Gameplay::Plays::OurCornerKick_ChipToGoalArea, "Restarts");

using namespace Geometry2d;
using namespace std;

OurCornerKick_ChipToGoalArea::OurCornerKick_ChipToGoalArea(GameplayModule* gameplay):
	Play(gameplay),
	_kicker(gameplay),
	_center1(gameplay),
	_center2(gameplay),
	_defender1(gameplay, Behaviors::Defender::Left),
	_defender2(gameplay, Behaviors::Defender::Right),
	_pdt(gameplay, &_kicker)
{
	_defender2.otherDefenders.insert(&_defender1);
	_defender1.otherDefenders.insert(&_defender1);

	_center1.target = Point(0.0, Field_Length / 2.0);

	_target = Segment(Point(0,Field_Length-Field_PenaltyDist), Point(0,Field_Length));

	_state = Setup;
}

float OurCornerKick_ChipToGoalArea::score(GameplayModule* gameplay)
{
	const GameState &gs = gameplay->state()->gameState;
	Point ballPos = gameplay->state()->ball.pos;
	bool chipper_available = false;
	for (OurRobot * r :  gameplay->playRobots())
	{
		if (r && r->hardwareVersion() == Packet::RJ2011)
		{
			chipper_available = true;
			break;
		}
	}
	return (gs.setupRestart() && gs.ourDirect() && chipper_available && ballPos.y > (Field_Length - 1.0)) ? 1 : INFINITY;
}

bool OurCornerKick_ChipToGoalArea::run()
{
	set<OurRobot*> available = _gameplay->playRobots();

	if(!assignNearestChipper(_kicker.robot, available, ball().pos))
	{
		return false;
	}

	assignNearest(_center1.robot, available, Point(-Field_Width/4.0, Field_Length));
	assignNearest(_center2.robot, available, Point( Field_Width/4.0, Field_Length));
	assignNearest(_defender1.robot, available, Geometry2d::Point(-Field_GoalHeight/2.0, 0.0));
	assignNearest(_defender2.robot, available, Geometry2d::Point( Field_GoalHeight/2.0, 0.0));

	_defender1.run();
	_defender2.run();

	if(ball().pos.x < 0)
	{
		ReceiveSetupPoint1 = Point(Field_ArcRadius + 0.5, Field_Length - Robot_Radius - 0.1);
		ReceiveSetupPoint2 = Point(Field_ArcRadius + 0.5, Field_Length - Robot_Radius - Robot_Diameter - 0.2);
	}
	else
	{
		ReceiveSetupPoint1 = Point(-Field_ArcRadius - 0.5, Field_Length - Robot_Radius - 0.1);
		ReceiveSetupPoint2 = Point(-Field_ArcRadius - 0.5, Field_Length - Robot_Radius - Robot_Diameter - 0.2);
	}

	// State transitions
	switch(_state)
	{
	case Setup:
		if(_center1.robot->pos.distTo(ReceiveSetupPoint1) <= 0.15 && _center2.robot->pos.distTo(ReceiveSetupPoint2) <= 0.15)
			_state = Kick;
		break;
	case Kick:
		if(_kicker.done() || !_pdt.keepRunning() || ball().vel.mag() > 1.0)
			_state = Receive;
		break;
	case Receive:
		if( ball().pos.distTo(_center1.robot->pos) <= Robot_Radius + 0.15 ||
			ball().pos.distTo(_center2.robot->pos) <= Robot_Radius + 0.15 ||
			ball().pos.y < Field_Length - 1.0)
				return false;
		break;
	}

	//Run
	switch(_state)
	{
	case Setup:
	{
		_kicker.robot->addText("CK State: Setup");
		_center1.target = ReceiveSetupPoint1;
		_center1.face = ball().pos;
		_center2.target = ReceiveSetupPoint2;
		_center2.face = ball().pos;
		_center1.run();
		_center2.run();
		_kicker.robot->move(ball().pos + Point( (ball().pos.x / abs(ball().pos.x)) * Robot_Diameter,0));
		_kicker.robot->face(ball().pos);
		break;
	}
	case Kick:
	{
		_kicker.robot->addText("CK State: Kick");
		// Setup kicker
		_kicker.setTarget(_target);
		_kicker.use_chipper = true;
		_kicker.use_line_kick = true;
		_kicker.minChipRange = 0;
		_kicker.maxChipRange = ball().pos.distTo(_target.center());

		_pdt.backoff.robots.clear();
		_pdt.backoff.robots.insert(_kicker.robot);
		_pdt.run();

		Point dir = ball().pos - _kicker.robot->pos;
		_center1.target = _kicker.robot->pos + dir.normalized() * (4.0/5.0) * Field_Width;
		_center2.target = _kicker.robot->pos + dir.normalized() * (4.0/5.0) * Field_Width;
		_center1.run();
		_center2.run();
		break;
	}
	case Receive:
	{
		_kicker.robot->addText("CK State: Receive");
		_kicker.robot->worldVelocity((ball().pos - _kicker.robot->pos).normalized() * 1.0);
		_kicker.robot->face(ball().pos);
		if(_center1.robot)
		{
			_center1.robot->worldVelocity((ball().pos - _center1.robot->pos).normalized() * 1.0);
			_center1.robot->face(ball().pos);
		}
		if(_center2.robot)
		{
			_center2.robot->worldVelocity((ball().pos - _center2.robot->pos).normalized() * 1.0);
			_center2.robot->face(ball().pos);
		}

		state()->drawCircle(_kicker.robot->pos, Robot_Radius + 0.15, QColor(255,0,255), "ChipToGoal");
		if(_center1.robot)
			state()->drawCircle(_center1.robot->pos, Robot_Radius + 0.15, QColor(255,0,255), "ChipToGoal");
		if(_center2.robot)
			state()->drawCircle(_center2.robot->pos, Robot_Radius + 0.15, QColor(255,0,255), "ChipToGoal");

		break;
	}
	}

	return true;
}

OurCornerKick_ChipToGoalArea::~OurCornerKick_ChipToGoalArea() {
}

} /* namespace Plays */
} /* namespace Gameplay */
