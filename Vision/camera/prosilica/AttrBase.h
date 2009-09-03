#pragma once

#include "PvApi.h"

class AttrBase
{
public:
    AttrBase(tPvHandle cam, const char *name);
    ~AttrBase();
    
    virtual void reset();
    
protected:
    tPvHandle cam;
    char *name;

private:
    AttrBase(const AttrBase &other);
    AttrBase &operator=(const AttrBase &other);
};
