#include "Offense.hpp"

using namespace std;
using namespace Geometry2d;

Gameplay::Plays::Offense::Offense(GameplayModule *gameplay):
	Play(gameplay, 4),
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
	if(!_kicker1.assign(available)){return false;};
	if(!_kicker2.assign(available)){return false;};
	if(!_fullback1.assign(available)){return false;};
	if(!_fullback2.assign(available)){return false;};

	_robots.insert(_kicker1.robot());
	_robots.insert(_kicker2.robot());
	_robots.insert(_fullback1.robot());
	_robots.insert(_fullback2.robot());

	// handle sides for fullbacks
	if (_fullback1.robot()->pos().x < _fullback2.robot()->pos().x) {
		_fullback1.side(Behaviors::Fullback::Left);
		_fullback2.side(Behaviors::Fullback::Right);
	} else {
		_fullback1.side(Behaviors::Fullback::Right);
		_fullback2.side(Behaviors::Fullback::Left);
	}

	return _robots.size() >= _minRobots;
}

bool Gameplay::Plays::Offense::run()
{
	// handle forward behavior
	// basic approach: closest robot gets the ball, the other robot goes to the other side of the field

	Point ballPos = ball().pos;

	Robot * other;
	if (_kicker1.robot()->pos().distTo(ballPos) < _kicker2.robot()->pos().distTo(ballPos)) {
		_kicker1.run();
		other = _kicker2.robot();
	} else {
		_kicker2.run();
		other = _kicker2.robot();
	}
	// drive to other side of the field
	float lag_y_dist = 0.5, x_offset = 1.5;
	float newX = (ballPos.x < 0.0) ? ballPos.x + x_offset : ballPos.x - x_offset;
	other->move(Point(newX, ballPos.y - lag_y_dist), false);

	// run standard fullback behavior
	_fullback1.run();
	_fullback2.run();
	
	return true;
}
