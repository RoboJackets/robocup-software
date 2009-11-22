#include "Robot.hpp"
#include "GameplayModule.hpp"

Gameplay::Robot::Robot(GameplayModule *gameplay, int id, bool self)
{
	_gameplay = gameplay;
	_id = id;
	_self = self;
	
	willKick = false;
	avoidBall = false;
	
	for (int i = 0; i < Constants::Robots_Per_Team; ++i)
	{
		approachOpponent[i] = false;
	}

	if (_self)
	{
		_packet = &_gameplay->state()->self[_id];
	} else {
		_packet = &_gameplay->state()->opp[_id];
	}
}

void Gameplay::Robot::resetMotionCommand()
{
	packet()->cmd = Packet::MotionCmd();
	
	// Stay in place if possible.
	move(pos());
}
