#pragma once

#include "AttrBase.h"

#include <QComboBox>

class AttrEnum: public QComboBox, public AttrBase
{
    Q_OBJECT;

public:
    AttrEnum(tPvHandle cam, const char *name);

    void reset();
    
protected Q_SLOTS:
    void change();
};
