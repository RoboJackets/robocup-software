#include "Role.hpp"
#include "Play.hpp"
#include "Behavior.hpp"

#include <boost/foreach.hpp>

Gameplay::Role::Role(Play *play, const std::string &name)
{
    _robot = 0;
    _play = play;
    _name = name;
    
    _current = 0;
    _play->_roles.push_back(this);
}

Gameplay::Role::~Role()
{
    BOOST_FOREACH(Behavior *behavior, _behaviors)
    {
        delete behavior;
    }
    
    _play->_roles.remove(this);
}

void Gameplay::Role::robot(Robot *robot)
{
    if (_robot)
    {
        stop();
        _robot->role(0);
    }
    
    _robot = robot;
    
    if (_robot)
    {
        _robot->role(this);
        start();
    }
}

Gameplay::Robot *Gameplay::Role::selectRobot()
{
    // Select a robot for this role
    Behavior *bestBehavior = 0;
    float bestPriority = 0;
    BOOST_FOREACH(Behavior *behavior, _behaviors)
    {
        float p = behavior->priority();
        if (!bestBehavior || p > bestPriority)
        {
            bestPriority = p;
            bestBehavior = behavior;
        }
    }
    
    if (bestBehavior)
    {
        // The behavior with the highest priority selects the robot.
        return bestBehavior->selectRobot();
    } else {
        // There weren't any behaviors, so we can't choose a robot.
        return 0;
    }
    
}

void Gameplay::Role::start()
{
    _current = 0;
    
    if (!_behaviors.empty())
    {
        _behaviors[0]->robot(_robot);
    }
}

void Gameplay::Role::advance()
{
    if (!lastBehavior())
    {
        _behaviors[_current]->robot(0);
        ++_current;
        _behaviors[_current]->robot(_robot);
    }
}

void Gameplay::Role::stop()
{
    Behavior *behavior = currentBehavior();
    if (behavior)
    {
        behavior->robot(0);
    }
}

bool Gameplay::Role::done() const
{
    Behavior *behavior = currentBehavior();
    return !behavior || (lastBehavior() && behavior->done());
}
