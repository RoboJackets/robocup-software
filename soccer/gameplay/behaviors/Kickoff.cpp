#include "Kickoff.hpp"

Gameplay::Behaviors::Kickoff::Kickoff(GameplayModule *gameplay):
    SingleRobotBehavior(gameplay),
    kick(gameplay)
{
}

bool Gameplay::Behaviors::Kickoff::run()
{
	kick.robot = robot;
    if (!robot || !robot->visible)
    {
	return false;
    }
    
    // Use the real ball position if we have it.  Otherwise, assum the middle of the field.
    Geometry2d::Point ballPos(0, Field_Length / 2);
    if (ball().valid)
    {
	ballPos = ball().pos;
    }
    
    switch (gameState().state)
    {
	case GameState::Setup:
	    robot->move(Geometry2d::Point(0,Field_Length / 2 - 0.3), false); // stop at end enabled
	    robot->face(ballPos);
	    
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
