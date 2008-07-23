#ifndef _PREDICATE_H_
#define _PREDICATE_H_

#include <map>
#include <string>

#include "Named_Object.hpp"

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

#endif // _PREDICATE_H_
