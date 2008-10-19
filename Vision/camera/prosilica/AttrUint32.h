#ifndef _ATTR_UINT32_H_
#define _ATTR_UINT32_H_

#include "AttrBase.h"

#include <QSpinBox>

class AttrUint32: public QSpinBox, public AttrBase
{
    Q_OBJECT;
    
public:
    AttrUint32(tPvHandle cam, const char *name);

    void reset();

protected Q_SLOTS:
    void change();
};

#endif // _ATTR_UINT32_H_
