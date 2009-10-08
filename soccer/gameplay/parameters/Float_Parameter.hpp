#pragma once

#include "Parameter.hpp"

namespace Gameplay
{    
    class Float_Parameter: public Parameter
    {
    public:
        Float_Parameter(Behavior *behavior, const char *name, float def = 0);
        
        virtual void clear();
        virtual void set(float value);
        
        float value() const { return _value; }
        
    protected:
        float _value;
        float _default_value;
    };
}
