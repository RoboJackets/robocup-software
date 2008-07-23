#ifndef _ATTR_BASE_H_
#define _ATTR_BASE_H_

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

#endif // _ATTR_BASE_H_
