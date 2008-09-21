#include "AttrBase.h"

#include <string.h>
#include <stdlib.h>

AttrBase::AttrBase(tPvHandle cam, const char *name)
{
    this->cam = cam;
    this->name = strdup(name);
}

AttrBase::~AttrBase()
{
    if (name)
    {
        free(name);
        name = 0;
    }
}

void AttrBase::reset()
{
}
