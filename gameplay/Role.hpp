#pragma once

#include "Robot.hpp"

#include <string>
#include <vector>

namespace Gameplay
{
    class Play;
    class Behavior;

    class Role
    {
    public:
        Role(Play *play, const std::string &name);
        ~Role();
        
        // Selects the best robot for this role.
        // This find the highest priority behavior in this role and
        // asks that behavior to select the robot.
        Robot *selectRobot();
        
        // Starts the first behavior.
        void start();
        
        // Runs the current behavior.
        void run();
        
        // Advances to the next behavior.
        void advance();
        
        // Stops the current behavior.
        void stop();
        
        // Returns true if this role is on its last behavior and that behavior is done.
        bool done() const;
        
        bool lastBehavior() const
        {
            // Don't use (_behaviors.size() - 1) because size is unsigned.
            return (_current + 1) == _behaviors.size();
        }
        
        // Returns the current behavior or 0 if none.
        Behavior *currentBehavior() const
        {
            if (_current < _behaviors.size())
            {
                return _behaviors[_current];
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
        
        const std::vector<Behavior *> behaviors() const { return _behaviors; }
        
        // Returns the index in behaviors() of the current behavior.
        unsigned int current() const { return _current; }
        
        // Which robot is assigned to this role, or null if none.
        Robot *robot() const { return _robot; }
        
    protected:
        friend class Behavior;

        Play *_play;
        std::string _name;
        std::vector<Behavior *> _behaviors;
        Robot *_robot;
        unsigned int _current;
    };
}
