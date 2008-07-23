#ifndef _TACTICS__FLOAT_PARAMETER_HPP_
#define _TACTICS__FLOAT_PARAMETER_HPP_

#include "Parameter.hpp"

namespace Tactics
{    
    class Float_Parameter: public Parameter
    {
    public:
        Float_Parameter(Base *tactic, const char *name, float def = 0);
        
        virtual void clear();
        virtual void set(float value);
        
        float value() const { return _value; }
        
    protected:
        float _value;
        float _default_value;
    };
}

#endif // _TACTICS__FLOAT_PARAMETER_HPP_
