#ifndef _ATTR_FLOAT_H_
#define _ATTR_FLOAT_H_

#include "AttrBase.h"

#include <QLineEdit>

class AttrFloat: public QLineEdit, public AttrBase
{
    Q_OBJECT;
    
public:
    AttrFloat(tPvHandle cam, const char *name);

    void reset();
    
protected Q_SLOTS:
    void change();
};

#endif // _ATTR_FLOAT_H_
