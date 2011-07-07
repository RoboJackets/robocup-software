#include "OurKickoff.hpp"

#include <stdio.h>

using namespace std;

REGISTER_PLAY_CATEGORY(Gameplay::Plays::OurKickoff, "Restarts")

namespace Gameplay
{
	namespace Plays
	{
		REGISTER_CONFIGURABLE(OurKickoff)
	}
}

void Gameplay::Plays::OurKickoff::createConfiguration(Configuration *cfg)
{

}

Gameplay::Plays::OurKickoff::OurKickoff(GameplayModule *gameplay):
	Play(gameplay),
	_kicker(gameplay),
	_idle1(gameplay),
	_idle2(gameplay),
	_idle3(gameplay),
	_pdt(gameplay, &_kicker)
{
	_idle1.target = _gameplay->centerMatrix() * Geometry2d::Point(0.7, -0.2);
	_idle2.target = _gameplay->centerMatrix() * Geometry2d::Point(-0.7, -0.2);
	_idle3.target = Geometry2d::Point(0, 1.5);

	// FIXME: find a better way of selecting targets - using endline to give choice
//	_kicker.kick.setTargetGoal();
	_kicker.kickTarget(Geometry2d::Segment(Geometry2d::Point(-Field_Width/2.0, Field_Length),
											   Geometry2d::Point( Field_Width/2.0, Field_Length)));
}

float Gameplay::Plays::OurKickoff::score (Gameplay::GameplayModule* gameplay)
{
	const GameState &gs = gameplay->state()->gameState;
	return (gs.setupRestart() && gs.ourKickoff()) ? 0 : INFINITY;
}

bool Gameplay::Plays::OurKickoff::run()
{
	set<OurRobot *> available = _gameplay->playRobots();
	
	bool chipper_available = true;
	if (!assignNearestChipper(_kicker.robot, available, ball().pos))
	{
		chipper_available = false;
		assignNearestKicker(_kicker.robot, available, ball().pos);
	}

	_pdt.backoff.robots.clear();
	_pdt.backoff.robots.insert(_kicker.robot);
	assignNearest(_idle1.robot, available, _idle1.target);
	assignNearest(_idle2.robot, available, _idle2.target);
	assignNearest(_idle3.robot, available, _idle3.target);
	
	_idle1.face = ball().pos;
	_idle2.face = ball().pos;
	_idle3.face = ball().pos;
	
	_pdt.run();

	if (_pdt.kicked())
	{
		_idle1.target = ball().pos;
	}

	_idle1.run();
	_idle2.run();
	_idle3.run();
	
	return _pdt.keepRunning();
}
