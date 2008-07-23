#ifndef _TACTICS__ROBOT_PARAMETER_HPP_
#define _TACTICS__ROBOT_PARAMETER_HPP_

#include "Parameter.hpp"

class Robot;

namespace Tactics
{    
    class Robot_Parameter: public Parameter
    {
    public:
        Robot_Parameter(Base *tactic, const char *name);
        
        virtual void clear();
        virtual void set(Robot *value);
        
        Robot *robot() const { return _robot; }
        bool valid() const;
        
    protected:
        Robot *_robot;
    };
}

#endif // _TACTICS__ROBOT_PARAMETER_HPP_
