#ifndef _ROLE_H_
#define _ROLE_H_

#include "Robot.hpp"

#include <string>
#include <vector>

namespace Tactics
{
    class Base;
};

class Play;

class Role
{
public:
    Role(Play *play, const std::string &name);
    ~Role();
    
    Robot *select_robot();
    
    // Starts the first tactic.
    void start();
    
    // Runs the current tactic.
    void run();
    
    // Advances to the next tactic.
    void advance();
    
    // Stops the current tactic.
    void stop();
    
    // Returns true if this role is on its last tactic and that tactic is done.
    bool done() const;
    
    bool last_tactic() const
    {
        // Don't use (_tactics.size() - 1) because size is unsigned.
        return (_current + 1) == _tactics.size();
    }
    
    // Returns the current tactic or 0 if none.
    Tactics::Base *current_tactic() const
    {
        if (_current < _tactics.size())
        {
            return _tactics[_current];
        } else {
            return 0;
        }
    }
    
    // Returns true if this role is assigned a robot.
    bool assigned() const { return _robot != 0; }
    
    void robot(Robot *robot);
    
    Play *play() const { return _play; }
    
    const std::string &name() const { return _name; }
    void name(const std::string &value)
    {
        _name = value;
    }
    
    const std::vector<Tactics::Base *> tactics() const { return _tactics; }
    
    // Returns the index in tactics() of the current tactic.
    unsigned int current() const { return _current; }
    
    // Which robot is assigned to this role, or null if none.
    Robot *robot() const { return _robot; }
    
protected:
    friend class Tactics::Base;

    Play *_play;
    std::string _name;
    std::vector<Tactics::Base *> _tactics;
    Robot *_robot;
    unsigned int _current;
};

// Information about a parameter that takes a role.
// These are used by the play parser to handle forward references.
namespace Tactics
{
    class Parameter;
}

#endif // _ROLE_H_
