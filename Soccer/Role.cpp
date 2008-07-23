#include "Role.hpp"
#include "Play.hpp"
#include "tactics/Tactics.hpp"

#include <boost/foreach.hpp>

Packet::MotionCmd motion;
Packet::VisionData vision;

Role::Role(Play *play, const std::string &name)
{
    _robot = 0;
    _play = play;
    _name = name;
    
    _current = 0;
    _play->_roles.push_back(this);
}

Role::~Role()
{
    BOOST_FOREACH(Tactics::Base *tactic, _tactics)
    {
        delete tactic;
    }
    
    _play->_roles.remove(this);
}

void Role::robot(Robot *robot)
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

Robot *Role::select_robot()
{
    // Select a robot for this role
    Tactics::Base *best_tactic = 0;
    float best_priority = 0;
    BOOST_FOREACH(Tactics::Base *tactic, _tactics)
    {
        float p = tactic->priority();
        if (!best_tactic || p > best_priority)
        {
            best_priority = p;
            best_tactic = tactic;
        }
    }
    
    if (best_tactic)
    {
        // The tactic with the highest priority selects the robot.
        return best_tactic->select_robot();
    } else {
        // There weren't any tactics, so we can't choose a robot.
        return 0;
    }
    
}

void Role::start()
{
    _current = 0;
    
    if (!_tactics.empty())
    {
        _tactics[0]->robot(_robot);
    }
}

void Role::advance()
{
    if (!last_tactic())
    {
        _tactics[_current]->robot(0);
        ++_current;
        _tactics[_current]->robot(_robot);
    }
}

void Role::stop()
{
    Tactics::Base *tactic = current_tactic();
    if (tactic)
    {
        tactic->robot(0);
    }
}

bool Role::done() const
{
    Tactics::Base *tactic = current_tactic();
    return !tactic || (last_tactic() && tactic->done());
}
