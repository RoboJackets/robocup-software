#include "OurKickoff.hpp"

using namespace std;

Gameplay::Plays::OurKickoff::OurKickoff(GameplayModule *gameplay):
	Play(gameplay),
	_kicker(gameplay),
	_idle1(gameplay),
	_idle2(gameplay),
	_idle3(gameplay),
	_pdt(gameplay, &_kicker)
{
}

bool Gameplay::Plays::OurKickoff::applicable()
{
	return (gameState().setupRestart() && gameState().ourKickoff()) || _pdt.keepRunning();
}

bool Gameplay::Plays::OurKickoff::assign(set<Robot *> &available)
{
	_robots = available;
	
	_idle1.target = _gameplay->centerMatrix() * Geometry2d::Point(0.7, -0.2);
	_idle2.target = _gameplay->centerMatrix() * Geometry2d::Point(-0.7, -0.2);
	_idle3.target = Geometry2d::Point(0, 1.5);
	
	_pdt.assign(available);
	_idle1.assign(available);
	_idle2.assign(available);
	_idle3.assign(available);
	
	if (_kicker.assigned() && _idle1.assigned())
	{
	    printf("Kickoff: target %d\n", _idle1.robot()->id());
	    _kicker.kick.setTarget(_idle1.robot());
	    _kicker.kick.targetType(Behaviors::Kick::ROBOT);
	} else {
	    printf("Kickoff: target goal\n");
	    _kicker.kick.setTarget();
	}

	return _robots.size() >= _minRobots;
}

bool Gameplay::Plays::OurKickoff::run()
{
	_idle1.face = ball().pos;
	_idle2.face = ball().pos;
	_idle3.face = ball().pos;
	
	_pdt.run();
	_idle1.run();
	_idle2.run();
	_idle3.run();
	
	return true;
}
