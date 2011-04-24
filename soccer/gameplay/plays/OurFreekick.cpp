#include "OurFreekick.hpp"

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::OurFreekick, "Restarts")

Gameplay::Plays::OurFreekick::OurFreekick(GameplayModule *gameplay):
	Play(gameplay),
	_kicker(gameplay),
	_center(gameplay),
	_fullback1(gameplay, Behaviors::Fullback::Left),
	_fullback2(gameplay, Behaviors::Fullback::Right),
	_pdt(gameplay, &_kicker)
{
	_center.target = _gameplay->centerMatrix() * Geometry2d::Point(0, 1.5);

	// FIXME: find a better setting for kicking
	// this target is an expanded version of the goal to give more options
	_kicker.setTarget(Geometry2d::Segment(Geometry2d::Point(-Field_Width/3.0, Field_Length),
										  Geometry2d::Point( Field_Width/3.0, Field_Length)));

	_fullback2.otherFullbacks.insert(&_fullback1);
	_fullback1.otherFullbacks.insert(&_fullback2);
}

float Gameplay::Plays::OurFreekick::score ( Gameplay::GameplayModule* gameplay )
{
	const GameState &gs = gameplay->state()->gameState;
	return (gs.setupRestart() && gs.ourFreeKick()) ? 0 : INFINITY;
}

bool Gameplay::Plays::OurFreekick::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	
	assignNearest(_kicker.robot, available, ball().pos);
	_pdt.backoff.robots.clear();
	_pdt.backoff.robots.insert(_kicker.robot);
	assignNearest(_center.robot, available, _center.target);
	assignNearest(_fullback1.robot, available, Geometry2d::Point(-Field_GoalHeight/2.0, 0.0));
	assignNearest(_fullback2.robot, available, Geometry2d::Point( Field_GoalHeight/2.0, 0.0));
	
	//FIXME: remove hack when new robots can kick reliably
	// swap robots so that striker is a 2008 robot
	if (_kicker.robot->newRevision()) {
		if (_center.robot)
			swap(_kicker.robot, _center.robot);
	}

// 	_kicker.aimType(Behaviors::Kick::ONETOUCH);
// 	_kicker.setVScale(0.3, 0.2); // drive slowly until close to ball
	_pdt.run();
	_center.run();
	_fullback1.run();
	_fullback2.run();
	
	return _pdt.keepRunning();
}
