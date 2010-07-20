#include "MixedChallenge2010.hpp"

#include <boost/foreach.hpp>

using namespace std;
using namespace Geometry2d;

REGISTER_PLAY(Gameplay::Plays::MixedChallenge2010)

Gameplay::Plays::MixedChallenge2010::MixedChallenge2010(GameplayModule *gameplay):
	Play(gameplay, 1),
	_state(STOPPED),
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

	return _robots.size() >= _minRobots;
}

bool Gameplay::Plays::MixedChallenge2010::run()
{
	const Point textOffset(-Constants::Robot::Radius*1.3, 0.0);

	if (_kicker1.assigned() && _kicker2.assigned()) {
		// determine which robots are not ours
		Robot * coRobotA = 0, * coRobotB = 0;
		BOOST_FOREACH(Robot * r, _gameplay->self) {
			if (r->exclude) {
				if (coRobotA == 0)
					coRobotA = r;
				else if (coRobotB == 0)
					coRobotB = r;
			}
		}

		// always force to setup if we are stopped
		if (!_gameplay->state()->gameState.playing())
			_state = STOPPED;

		// pull out useful info
		Point pos1 = _kicker1.robot()->pos();
		Point pos2 = _kicker2.robot()->pos();
		Point coPosA = coRobotA->pos();
		Point coPosB = coRobotB->pos();
		Point ballPos = ball().pos;

		// find closest of our robots
		Robot * closestOurs = (pos1.distTo(ballPos) < pos2.distTo(ballPos)) ? _kicker1.robot() : _kicker2.robot();
//		Robot * furthestOurs = (pos1.distTo(ballPos) >= pos2.distTo(ballPos)) ? _kicker1.robot() : _kicker2.robot();
		float closestDistOurs = (pos1.distTo(ballPos) < pos2.distTo(ballPos)) ? pos1.distTo(ballPos) : pos2.distTo(ballPos);

//		Robot * closestTheirs = (coPosA.distTo(ballPos) < coPosB.distTo(ballPos)) ? coRobotA : coRobotB;
//		Robot * furthestTheirs = (coPosA.distTo(ballPos) > coPosB.distTo(ballPos)) ? coRobotA : coRobotB;
		float closestDistTheirs = (coPosA.distTo(ballPos) < coPosB.distTo(ballPos)) ? coPosA.distTo(ballPos) : coPosB.distTo(ballPos);

		Robot * curTargetRobot = 0;

		switch (_state) {
		case STOPPED: {
			if (_gameplay->state()->gameState.playing()) {
				_state = SETUP;
			} else {
				Segment setupLine(Point(-Constants::Field::Width/2.0, 0.5),
						Point(Constants::Field::Width/2.0, 0.5));
				_kicker1.robot()->move(setupLine.nearestPoint(pos1));
				_kicker2.robot()->move(setupLine.nearestPoint(pos2));
			}
		}
		case SETUP: {
			Point kicker1Dest(-Constants::Field::Width/4.0, Constants::Field::Length/3.0); // near, left
			Point kicker2Dest(-Constants::Field::Width/4.0, 2.0*Constants::Field::Length/3.0); // far, left

			_kicker1.robot()->move(kicker1Dest);
			_kicker1.robot()->face(ballPos);
			_kicker2.robot()->move(kicker2Dest);
			_kicker2.robot()->face(ballPos);

			float setup_thresh = 1.0;
			if (pos1.nearPoint(kicker1Dest, setup_thresh) && pos2.nearPoint(kicker2Dest, setup_thresh)) {
				// switching logic out of this state
				if (closestDistOurs < closestDistTheirs) {
					// assign behavior to kick
					if (closestOurs == _kicker1.robot()) {
						// go kick
						_kicker1.run();
						_usingKicker1 = true;
						_usingKicker2 = false;
						Point robotDestNear(-pos1.x, Constants::Field::Length - pos1.y);
						curTargetRobot = (robotDestNear.distTo(coPosA) < robotDestNear.distTo(coPosB)) ? coRobotA : coRobotB;
						_kicker1.setTarget(curTargetRobot);
					} else {
						// go kick
						_kicker2.run();
						_usingKicker1 = false;
						_usingKicker2 = true;
						Point robotDestNear(-pos2.x, Constants::Field::Length - pos2.y);
						curTargetRobot = (robotDestNear.distTo(coPosA) < robotDestNear.distTo(coPosB)) ? coRobotA : coRobotB;
						_kicker2.setTarget(curTargetRobot);
					}
					// pass state - go fetch ball
					_state = PASSING;
				} else {
					// wait on pass
					_state = RECEIVING;
				}
			}
		}
		case PASSING: {
			// execute
			bool kickerDone = false;
			if (_usingKicker1)
				kickerDone = _kicker1.run();
			if (_usingKicker2)
				kickerDone = _kicker2.run();

			// transition
			if (kickerDone)
				_state = WAITONPASS;
		}
		case WAITONPASS: {

		}
		case RECEIVING: {
			_kicker1.robot()->face(ballPos);
			_kicker2.robot()->face(ballPos);

			Point fieldCenter(0.0, Constants::Field::Length/2.0);
			_kicker1.robot()->move(fieldCenter);
			_kicker2.robot()->move(fieldCenter);

			// transition out - when we receive the pass
//			if () {
//
//			}
		}
		case SHOOTING: {

		}
		}

	}
	return true;
}
