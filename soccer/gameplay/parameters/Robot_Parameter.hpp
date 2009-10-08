#pragma once

#include "Parameter.hpp"

namespace Gameplay
{    
    class Robot;
    
    class Robot_Parameter: public Parameter
    {
    public:
        Robot_Parameter(Behavior *tactic, const char *name);
        
        virtual void clear();
        virtual void set(Robot *value);
        
        Robot *robot() const { return _robot; }
        bool valid() const;
        
    protected:
        Robot *_robot;
    };
}
