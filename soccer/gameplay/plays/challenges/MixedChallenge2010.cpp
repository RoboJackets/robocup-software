#include "MixedChallenge2010.hpp"

#include <boost/foreach.hpp>

using namespace std;
using namespace Geometry2d;

Gameplay::Plays::MixedChallenge2010::MixedChallenge2010(GameplayModule *gameplay):
	Play(gameplay, 1),
	_fieldSide(LEFT),   // *************************** Change here for side!
	_state(SETUP),
	_kicker1(gameplay),
	_kicker2(gameplay)
{
}

bool Gameplay::Plays::MixedChallenge2010::applicable()
{
	return true;
}

bool Gameplay::Plays::MixedChallenge2010::assign(set<Robot *> &available)
{
	_robots = available;
	
	_kicker1.assign(available);
	_kicker2.assign(available);
	Point ballPos = ball().pos;

	if (_kicker1.assigned() && _kicker2.assigned())
	{
		// set which robot is the kicker first
		_usingKicker1 = _kicker1.robot()->pos().distTo(ballPos) < _kicker2.robot()->pos().distTo(ballPos);
	} else {
		_usingKicker1 = true;
	}

	return _robots.size() >= _minRobots;
}

bool Gameplay::Plays::MixedChallenge2010::run()
{
	const Point textOffset(-Constants::Robot::Radius*1.3, 0.0);

//	// determine which robots are not ours
//	Robot * coRobotA = 0, * coRobotB = 0;
//	BOOST_FOREACH(Robot * r, _gameplay->self) {
//		if (r->exclude) {
//			if (coRobotA == 0)
//				coRobotA = r;
//			else if (coRobotB == 0)
//				coRobotB = r;
//		}
//	}




//	// set kickers to onetouch for snap shots
//	_kicker1.aimType(Behaviors::Kick::ONETOUCH);
//	_kicker2.aimType(Behaviors::Kick::ONETOUCH);
//
//	// handle forward behavior
//	// basic approach: closest robot gets the ball, the other robot goes to the other side of the field
//	Point ballPos = ball().pos;
//
//	// manually reset any kickers
//	if (_kicker1.isDone())
//		_kicker1.restart();
//	if (_kicker2.isDone())
//		_kicker2.restart();
//
//	if (_kicker1.assigned() && _kicker2.assigned())
//	{
//		float kick1Dist, kick2Dist;
//
//		Point ballPos = ball().pos;
//
//		Robot * other;
//		kick1Dist = _kicker1.robot()->pos().distTo(ballPos);
//		kick2Dist = _kicker2.robot()->pos().distTo(ballPos);
//
//		// handle toggle cases
//		float percent = 0.85;
//		if (kick1Dist < percent * kick2Dist) {
//			if (!_usingKicker1 || kick1Dist > 1.0) {
//				_kicker1.restart();
//			}
//			other = _kicker2.robot();
//			_usingKicker1 = true;
//		} else if (kick2Dist < percent * kick1Dist) {
//			if (_usingKicker1 || kick2Dist > 1.0) {
//				_kicker2.restart();
//			}
//			other = _kicker1.robot();
//			_usingKicker1 = false;
//		}
//
//		// execute
//		Point kickerPos;
//		if (_usingKicker1) {
//			_kicker1.run();
//			drawText("Active", _kicker1.robot()->pos() + textOffset);
//			kickerPos = _kicker1.robot()->pos();
//			other = _kicker2.robot();
//		} else {
//			_kicker2.run();
//			drawText("Active", _kicker2.robot()->pos() + textOffset);
//			kickerPos = _kicker2.robot()->pos();
//			other = _kicker1.robot();
//		}
//
//		// drive to other side of the field
//		float lag_y_dist = 0.5, x_offset = 1.5;
//		float newX = (ballPos.x < 0.0) ? ballPos.x + x_offset : ballPos.x - x_offset;
//		float newY = (ballPos.y < Constants::Field::Length/2.0) ? ballPos.y + lag_y_dist : ballPos.y - lag_y_dist;
//		drawText("Backup", other->pos() + textOffset);
//		other->move(Point(newX, newY), false);
//		other->face(kickerPos);
//	} else {
//		_kicker1.run();
//		_kicker2.run();
//	}
	
	return true;
}
