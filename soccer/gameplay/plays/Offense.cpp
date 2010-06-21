#include "Offense.hpp"

using namespace std;
using namespace Geometry2d;

Gameplay::Plays::Offense::Offense(GameplayModule *gameplay):
	Play(gameplay, 1),
	_fullback1(gameplay),
	_fullback2(gameplay),
	_kicker1(gameplay),
	_kicker2(gameplay)
{
}

bool Gameplay::Plays::Offense::applicable()
{
	bool refApplicable =_gameplay->state()->gameState.playing();
	bool gameplayApplicable = _gameplay->state()->stateID.posession == Packet::LogFrame::OFFENSE ||
						      _gameplay->state()->stateID.posession == Packet::LogFrame::FREEBALL;

	return refApplicable && gameplayApplicable;
}

bool Gameplay::Plays::Offense::assign(set<Robot *> &available)
{
	_robots = available;
	
	_kicker1.assign(available);
	_kicker2.assign(available);
	_fullback1.assign(available);
	_fullback2.assign(available);
	Point ballPos = ball().pos;

	// handle sides for fullbacks
	if (_fullback1.assigned() && _fullback2.assigned())
	{
		if (_fullback1.robot()->pos().x < _fullback2.robot()->pos().x) {
			_fullback1.side(Behaviors::Fullback::Left);
			_fullback2.side(Behaviors::Fullback::Right);
		} else {
			_fullback1.side(Behaviors::Fullback::Right);
			_fullback2.side(Behaviors::Fullback::Left);
		}

		// handle sides for fullbacks
		if (_fullback1.robot()->pos().x < _fullback2.robot()->pos().x) {
			_fullback1.side(Behaviors::Fullback::Left);
			_fullback2.side(Behaviors::Fullback::Right);
		} else {
			_fullback1.side(Behaviors::Fullback::Right);
			_fullback2.side(Behaviors::Fullback::Left);
		}
	}

	if (_kicker1.assigned() && _kicker2.assigned())
	{
		// set which robot is the kicker first
		_usingKicker1 = _kicker1.robot()->pos().distTo(ballPos) < _kicker2.robot()->pos().distTo(ballPos);
	} else {
		_usingKicker1 = true;
	}

	return _robots.size() >= _minRobots;
}

bool Gameplay::Plays::Offense::run()
{
	// handle forward behavior
	// basic approach: closest robot gets the ball, the other robot goes to the other side of the field
	Point ballPos = ball().pos;

	if (_kicker1.assigned() && _kicker2.assigned())
	{
		float kick1Dist, kick2Dist;

		Point ballPos = ball().pos;

		Robot * other;
		kick1Dist = _kicker1.robot()->pos().distTo(ballPos);
		kick2Dist = _kicker2.robot()->pos().distTo(ballPos);

		// handle toggle cases
		float percent = 0.85;
		if (kick1Dist < percent * kick2Dist) {
			if (!_usingKicker1) {
				_kicker1.restart();
			}
			_kicker1.run();
			other = _kicker2.robot();
			_usingKicker1 = true;
		} else if (kick2Dist < percent * kick1Dist) {
			if (_usingKicker1) {
				_kicker2.restart();
			}
			_kicker2.run();
			other = _kicker1.robot();
			_usingKicker1 = false;
		} else if (_usingKicker1) {
			_kicker1.run();
			other = _kicker2.robot();
		} else {
			_kicker2.run();
			other = _kicker1.robot();
		}

		// drive to other side of the field
		float lag_y_dist = 0.5, x_offset = 1.5;
		float newX = (ballPos.x < 0.0) ? ballPos.x + x_offset : ballPos.x - x_offset;
		float newY = (ballPos.y < Constants::Field::Length/2.0) ? ballPos.y + lag_y_dist : ballPos.y - lag_y_dist;
		other->move(Point(newX, newY), false);
	} else {
		_kicker1.run();
		_kicker2.run();
	}

	// run standard fullback behavior
	_fullback1.run();
	_fullback2.run();
	
	return true;
}
