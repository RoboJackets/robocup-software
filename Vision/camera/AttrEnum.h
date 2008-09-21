#ifndef _ATTR_ENUM_H_
#define _ATTR_ENUM_H_

#include "AttrBase.h"

#include <QComboBox>

class AttrEnum: public QComboBox, public AttrBase
{
    Q_OBJECT;

public:
    AttrEnum(tPvHandle cam, const char *name);

    void reset();
    
protected slots:
    void change();
};

#endif // _ATTR_ENUM_H_
