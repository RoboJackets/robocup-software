#include "ZoneOffense.hpp"

#include <iostream>
#include <vector>
#include <boost/foreach.hpp>

using namespace std;
using namespace Geometry2d;

Gameplay::Behaviors::ZoneOffense::ZoneOffense(GameplayModule *gameplay)
: Behavior(gameplay, 2),
  _activeZone(NONE)
{
	_leftAttack = 0;
	_rightAttack = 0;
	_midfielder = 0;
	_kicker = 0;
	_markLeft = 0;
	_markRight = 0;
	_markMidfield = 0;

	const Point leftFarCorner(-Constants::Field::Width/2.0, Constants::Field::Length),
			rightFarCorner(Constants::Field::Width/2.0, Constants::Field::Length),
			centerFar(0.0, Constants::Field::Length);

	const Point	leftFarThird(-Constants::Field::Width/2.0, Constants::Field::Length*(2.0/3.0)),
			rightFarThird(Constants::Field::Width/2.0, Constants::Field::Length*(2.0/3.0)),
			centerFarThird(0.0, Constants::Field::Length*(2.0/3.0));

	const Point	leftHomeThird(-Constants::Field::Width/2.0, Constants::Field::Length/3.0),
			rightHomeThird(Constants::Field::Width/2.0, Constants::Field::Length/3.0),
			centerHomeThird(0.0, Constants::Field::Length/3.0);

	const Point	leftMidHalf(-Constants::Field::Width/2.0, Constants::Field::Length/2.0),
			rightMidHalf(Constants::Field::Width/2.0, Constants::Field::Length/2.0),
			center(0.0, Constants::Field::Length/2.0);

	const Point	leftHomeBig(-Constants::Field::Width/2.0, Constants::Field::Length/1.7),
			rightHomeBig(Constants::Field::Width/2.0, Constants::Field::Length/1.7),
			centerHomeBig(0.0, Constants::Field::Length/1.7);

	const Point	centerLeftHome(-Constants::Field::Width/4.0, Constants::Field::Length/1.7),
			centerRightHome(Constants::Field::Width/4.0, Constants::Field::Length/1.7);

	// small zones
	_midfieldZone = Rect(leftMidHalf, rightHomeThird);
	_leftZone = Rect(leftFarCorner, center);
	_rightZone = Rect(centerFar, rightMidHalf);
	_leftNoMidZone = Rect(leftFarCorner, centerHomeThird);
	_rightNoMidZone = Rect(rightFarCorner, centerHomeThird);

	// big zones
	_midfieldBigZone = Rect(leftFarThird, rightHomeThird);
	_leftBigZone = Rect(leftFarCorner, centerHomeBig);
	_rightBigZone = Rect(rightFarCorner, centerHomeBig);
	_leftBigNoMidZone = Rect(leftFarCorner, centerRightHome);
	_rightBigNoMidZone = Rect(rightFarCorner, centerLeftHome);
}

Gameplay::Behaviors::ZoneOffense::~ZoneOffense() {
	//TODO release all behaviors here
}

bool Gameplay::Behaviors::ZoneOffense::assign(std::set<Robot *> &available) {
	cout << "ZoneOffense: assign()" << endl;
	takeAll(available);
	cout << "ZoneOffense: after takeall" << endl;


	if (_robots.size() == 2) {cout << "ZoneOffense: " << endl;
		cout << "ZoneOffense: got 2 robots" << endl;
		Robot *a = *_robots.begin(), *b = *(_robots.begin()++);
		if (a->pos().x < b->pos().x) {
			_leftAttack = a;
			_rightAttack = b;
		} else {
			_leftAttack = b;
			_rightAttack = a;
		}
	} else if (_robots.size() == 3) {
		cout << "ZoneOffense: got 3 robots" << endl;
		// find the midfielder
		float backY = 100;
		vector<Robot*> forwards;
		BOOST_FOREACH(Robot * r, _robots) {
			if (!r) {
				_midfielder = r;
				backY = _midfielder->pos().y;
			} else if (r->pos().y < backY) {
				backY = r->pos().y;
				forwards.push_back(_midfielder);
				_midfielder = r;
			}
		}
		cout << "ZoneOffense: 3 robot version - found " << forwards.size() << " forwards" << endl;

		// pick left and right
		Robot *a = forwards[0], *b = forwards[1];
		if (!a) cout << "ZoneOffense: robot a is not valid " << endl; // FIXME: this check fails
		if (!b) cout << "ZoneOffense: robot b is not valid " << endl;
		if (a->pos().x < b->pos().x) {
			_leftAttack = a;
			_rightAttack = b;
		} else {
			_leftAttack = b;
			_rightAttack = a;
		}

		cout << "ZoneOffense: set robot sides" << endl;
	} else {
		return false;
	}

	// additional setup
	cout << "ZoneOffense: Adding right mark" << endl;
	if (_markRight) delete _markRight;
	_markRight = new Mark(_gameplay);
	_markRight->assignOne(_rightAttack);

	cout << "ZoneOffense: Adding left mark" << endl;
	if (_markLeft) delete _markLeft;
	_markLeft = new Mark(_gameplay);
	_markLeft->assignOne(_leftAttack);

	if (_robots.size() == 3) {
		cout << "ZoneOffense: Adding midfield mark" << endl;
		if (_markMidfield) delete _markMidfield;
		_markMidfield = new Mark(_gameplay);
		_markMidfield->assignOne(_midfielder);
	}
	cout << "ZoneOffense: at end of assign" << endl;

	return true;
}

bool Gameplay::Behaviors::ZoneOffense::run() {

	// pull out state information and project the ball
	Point ballPos = ball().pos,
			ballVel = ball().vel,
			leftPos = _leftAttack->pos(),
			rightPos = _rightAttack->pos();
	//FIXME: get project right here

	// determine zone information for the ball for initialization
	if (_activeZone == NONE) {
		// handle midfielder
		if (_midfielder && ballPos.y < Constants::Field::Length * 0.5) {
			_activeZone = MIDFIELD;
			_kicker = new Kick(_gameplay);
			_kicker->assignOne(_midfielder);
			_kicker->aimType(Kick::ONETOUCH); // midfielder should just try to clear up fast
		}
		// get left and right
		else if (ballPos.x < 0.0) {
			_activeZone = LEFT;
			_kicker = new Kick(_gameplay);
			_kicker->assignOne(_leftAttack);
			_kicker->aimType(Kick::PIVOT);
		} else if (ballPos.x >= 0.0) {
			_activeZone = RIGHT;
			_kicker = new Kick(_gameplay);
			_kicker->assignOne(_rightAttack);
			_kicker->aimType(Kick::PIVOT);
		}
	}

	// determine if zones have switched
	bool switched = false;
	switch (_activeZone) {
	case MIDFIELD:
		switched = _midfieldBigZone.contains(ballPos);
		break;
	case LEFT:
		if (_midfielder)
			switched = _leftBigZone.contains(ballPos);
		else
			switched = _leftBigNoMidZone.contains(ballPos);
		break;
	case RIGHT:
		if (_midfielder)
		switched = _rightBigZone.contains(ballPos);
		else
			switched = _rightBigNoMidZone.contains(ballPos);
		break;
	default:
		break;
	}

	// change targets
	if (switched) {
		if (_midfielder && _midfieldZone.contains(ballPos)) {
			_activeZone = MIDFIELD;
			delete _kicker;
			_kicker = new Kick(_gameplay);
			_kicker->assignOne(_midfielder);
			_kicker->aimType(Kick::ONETOUCH);
		} else if ((_midfielder && _leftZone.contains(ballPos)) ||
				(!_midfielder && _leftNoMidZone.contains(ballPos))) {
			_activeZone = LEFT;
			delete _kicker;
			_kicker = new Kick(_gameplay);
			_kicker->assignOne(_leftAttack);
			_kicker->aimType(Kick::PIVOT);
		} else if ((_midfielder && _rightZone.contains(ballPos)) ||
				(!_midfielder && _rightNoMidZone.contains(ballPos))) {
			_activeZone = RIGHT;
			delete _kicker;
			_kicker = new Kick(_gameplay);
			_kicker->assignOne(_rightAttack);
			_kicker->aimType(Kick::PIVOT);
		}
	}

	// handle opponents
	vector<Robot *> leftOpps, rightOpps, midOpps;
	for (size_t i = 0; i<Constants::Robots_Per_Team; ++i) {
		Robot * op = _gameplay->opp[i];
		Point opPos = op->pos();
		if ((_midfielder && _rightZone.contains(opPos)) ||
				(!_midfielder && _rightNoMidZone.contains(opPos))) {
			rightOpps.push_back(op);
		} else if ((_midfielder && _leftZone.contains(opPos)) ||
				(!_midfielder && _leftNoMidZone.contains(opPos))) {
			leftOpps.push_back(op);
		} else if ((_midfielder && _rightZone.contains(opPos)) ||
				(!_midfielder && _rightNoMidZone.contains(opPos))) {
			rightOpps.push_back(op);
		}
	}

	// pick the closest opp to the ball to mark in each zone
	if (_activeZone != LEFT) {
		if (!leftOpps.empty()) {
			Robot * op = leftOpps.front();
			float bestDist = op->pos().distTo(ballPos);
			for (size_t i=1; i<leftOpps.size(); ++i) {
				float dist = opp(i)->pos().distTo(ballPos);
				if (dist < bestDist) {
					bestDist = dist;
					op = _gameplay->opp[i];
				}
			}
			_markLeft->markRobot(op);
			_markLeft->run();
		} else {
			_leftAttack->face(ballPos);
			_leftAttack->move(_leftZone.center());
		}
	}

	if (_activeZone != RIGHT) {
		if (!rightOpps.empty()) {
			Robot * op = rightOpps.front();
			float bestDist = op->pos().distTo(ballPos);
			for (size_t i=1; i<rightOpps.size(); ++i) {
				float dist = opp(i)->pos().distTo(ballPos);
				if (dist < bestDist) {
					bestDist = dist;
					op = _gameplay->opp[i];
				}
			}
			_markRight->markRobot(op);
			_markRight->run();
		} else {
			_rightAttack->face(ballPos);
			_rightAttack->move(_rightZone.center());
		}
	}

	if (_midfielder && _activeZone != MIDFIELD) {
		if (!midOpps.empty()) {
			Robot * op = midOpps.front();
			float bestDist = op->pos().distTo(ballPos);
			for (size_t i=1; i<midOpps.size(); ++i) {
				float dist = opp(i)->pos().distTo(ballPos);
				if (dist < bestDist) {
					bestDist = dist;
					op = _gameplay->opp[i];
				}
			}
			_markMidfield->markRobot(op);
			_markMidfield->run();
		}
		else {

			// try to stay on a line through center of zone
			Point zoneCenter = _midfieldZone.center();
			Segment zoneMidLine(Point(-Constants::Field::Width/2.0, zoneCenter.y),
								Point(Constants::Field::Width/2.0, zoneCenter.y));
			Segment ballRobotLine(ballPos, _midfielder->pos());
			Point dest = zoneCenter;
			zoneMidLine.intersects(ballRobotLine, &dest);

			_midfielder->face(ballPos);
			_midfielder->move(dest);
		}
	}

	// execute the kicker
	_kicker->run();

	return true;
}
