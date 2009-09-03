#pragma once

#include <map>
#include <string>

#include "Named_Object.hpp"

namespace Gameplay
{
    class Predicate: public Named_Object<Predicate>
    {
    public:
        Predicate(const char *name): Named_Object<Predicate>(name) {}

        bool evaluate() const
        {
            return value;
        }
        
        operator bool() const
        {
            return value;
        }
        
        bool operator = (bool b)
        {
            value = b;
            return b;
        }

        bool value;
    };
}
