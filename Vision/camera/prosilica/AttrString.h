#pragma once

#include "AttrBase.h"

#include <QLineEdit>

class AttrString: public QLineEdit, public AttrBase
{
    Q_OBJECT;

public:
    AttrString(tPvHandle cam, const char *name);

    void reset();

protected:
    char buf[256];

protected Q_SLOTS:
    void change();
};
