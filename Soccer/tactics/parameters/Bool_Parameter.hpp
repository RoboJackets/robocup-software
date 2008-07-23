#ifndef _TACTICS__BOOL_PARAMETER_HPP_
#define _TACTICS__BOOL_PARAMETER_HPP_

#include "Parameter.hpp"

namespace Tactics
{    
    class Bool_Parameter: public Parameter
    {
    public:
        Bool_Parameter(Base *tactic, const char *name, bool def = false);
        
        virtual void clear();
        virtual void set(float value);
        virtual void set(const std::string &str);
        
        bool value() const { return _value; }
        
    protected:
        bool _value;
        bool _default_value;
    };
}

#endif // _TACTICS__BOOL_PARAMETER_HPP_
