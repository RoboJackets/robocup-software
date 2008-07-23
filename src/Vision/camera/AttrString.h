#ifndef _ATTR_STRING_H_
#define _ATTR_STRING_H_

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

protected slots:
    void change();
};

#endif // _ATTR_STRING_H_
