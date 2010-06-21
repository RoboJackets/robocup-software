#include "Kickoff.hpp"

Gameplay::Behaviors::Kickoff::Kickoff(GameplayModule *gameplay):
    Behavior(gameplay),
    kick(gameplay)
{
}

bool Gameplay::Behaviors::Kickoff::assign(std::set< Gameplay::Robot* >& available)
{
    kick.assign(available);
    _robots.clear();
    if (kick.assigned())
    {
	_robots.insert(kick.robot());
    }
    
    return kick.assigned();
}

bool Gameplay::Behaviors::Kickoff::run()
{
    if (!allVisible())
    {
	return false;
    }
    
    // Use the real ball position if we have it.  Otherwise, assum the middle of the field.
    Geometry2d::Point ballPos(0, Constants::Field::Length / 2);
    if (ball().valid)
    {
	ballPos = ball().pos;
    }
    
    switch (gameState().state)
    {
	case GameState::Setup:
	    robot()->move(Geometry2d::Point(0,Constants::Field::Length / 2 - 0.3), false); // stop at end enabled
	    robot()->face(ballPos);
	    
	    // Need this in case the kickoff is restarted (transition from Ready to Setup).
	    // This should not normally happen but it helps with testing and sloppy referees.
	    kick.restart();
	    break;
	    
	case GameState::Ready:
	    kick.run();
	    break;
	    
	default:
	    break;
    }
    
    return gameState().state != GameState::Playing;
}
