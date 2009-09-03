#pragma once

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
