#pragma once

#include "Parameter.hpp"

namespace Gameplay
{    
    class Bool_Parameter: public Parameter
    {
    public:
        Bool_Parameter(Behavior *behavior, const char *name, bool def = false);
        
        virtual void clear();
        virtual void set(float value);
        virtual void set(const std::string &str);
        
        bool value() const { return _value; }
        
    protected:
        bool _value;
        bool _default_value;
    };
}
