#include "OurKickoff.hpp"

#include <boost/foreach.hpp>

using namespace std;

Gameplay::Plays::OurKickoff::OurKickoff(GameplayModule *gameplay):
	Play(gameplay),
	_kicker(gameplay),
	_idle1(gameplay),
	_idle2(gameplay),
	_idle3(gameplay),
	_backoff(gameplay)
{
	_preventDoubleTouch = false;
}

bool Gameplay::Plays::OurKickoff::applicable()
{
	return (gameState().setupRestart() && gameState().ourKickoff()) || _preventDoubleTouch;
}

bool Gameplay::Plays::OurKickoff::assign(set<Robot *> &available)
{
	_preventDoubleTouch = false;
	_robots = available;
	
	_idle1.target = _gameplay->centerMatrix() * Geometry2d::Point(0.7, -0.2);
	_idle2.target = _gameplay->centerMatrix() * Geometry2d::Point(-0.7, -0.2);
	_idle3.target = Geometry2d::Point(0, 1.5);
// 	_backoff.target = Geometry2d::Point(0, 2);
	
	_kicker.assign(available);
	_idle1.assign(available);
	_idle2.assign(available);
	_idle3.assign(available);
	
	if (_kicker.assigned())
	{
	    _backoff.assignOne(_kicker.robot());
	}
	
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
	if (_preventDoubleTouch && (gameState().state != GameState::Ready && gameState().state != GameState::Playing))
	{
	    // We were waiting for another robot to touch the ball but the referee has cancelled the kickoff.
	    // Allow applicable() to return false.
	    _preventDoubleTouch = false;
	}
	
	if (gameState().state == GameState::Ready)
	{
	    // At this point the kicker should either kick or collide with the ball.
	    // After that one other robot must touch it.
	    _preventDoubleTouch = true;
	}
	
	_idle1.face = ball().pos;
	_idle2.face = ball().pos;
	_idle3.face = ball().pos;
	
	if (_kicker.assigned())
	{
	    if (!_kicker.run())
	    {
    // 	    _preventDoubleTouch = false;
    // 	    _backoff.robot()->avoidBall = true;
		_backoff.run();
		printf("*** Kicker done\n");
	    }
	    
	    Robot *best = 0;
	    float bestDist = 0;
	    BOOST_FOREACH(Robot *r, _gameplay->self)
	    {
		float dist = r->pos().distTo(ball().pos);
		if (bestDist == 0 || bestDist > dist)
		{
		    bestDist = dist;
		    best = r;
		}
	    }
	    if (best != _kicker.robot())
	    {
		printf("Other better\n");
		_preventDoubleTouch = false;
	    }
	} else {
    	    _preventDoubleTouch = false;
	}
	
	_idle1.run();
	_idle2.run();
	_idle3.run();
	
	return true;
}
