#include "Robot.hpp"
#include "Role.hpp"
#include "Opponent.hpp"
#include "GameplayModule.hpp"

Gameplay::Robot::Robot(GameplayModule *gameplay, int id, bool self)
{
    _gameplay = gameplay;
    _free = true;
    _id = id;
    _self = self;
    _role = 0;
    _opponent = 0;
    _goalie = false;
    
    willKick = false;
    avoidBall = false;
    
    for (int i = 0; i < Constants::Robots_Per_Team; ++i)
    {
        approachOpponent[i] = false;
    }
}

Packet::LogFrame::Robot *Gameplay::Robot::state() const
{
    if (_self)
    {
        return &_gameplay->state()->self[_id];
    } else {
        return &_gameplay->state()->opp[_id];
    }
}

bool Gameplay::Robot::visible() const
{
    if (_self)
    {
        return _gameplay->state()->self[_id].valid;
    } else {
        return _gameplay->state()->opp[_id].valid;
    }
}

bool Gameplay::Robot::assigned() const
{
    return (_role && _role->assigned()) || _opponent || _goalie;
}

void Gameplay::Robot::role(Role *role)
{
    willKick = false;
    for (int i = 0; i < Constants::Robots_Per_Team; ++i)
    {
        approachOpponent[i] = false;
    }
    
    _role = role;
    
    if (_role)
    {
        _name = role->name();
    } else {
        _name.clear();
    }
}

void Gameplay::Robot::opponent(Opponent *opp)
{
    _opponent = opp;
    
    if (_opponent)
    {
        _name = opp->name();
    } else {
        _name.clear();
    }
}

Gameplay::Behavior *Gameplay::Robot::behavior() const
{
    if (_role)
    {
        return _role->currentBehavior();
    } else {
        return 0;
    }
}

void Gameplay::Robot::goalie(bool flag)
{
	_goalie = flag;
	if (flag)
	{
		_name = "goalie";
	} else {
		_name.clear();
	}
}
