#pragma once

// This camera generates a black frame for testing the vision framework.
// read_frame does not block.

#include "../Base.h"

namespace Camera
{
    class Dummy: public Base
    {
    public:
        Dummy();

        virtual QString name();
        virtual void open();
        virtual void close();
        virtual bool is_open();
        virtual QWidget *configuration();
        virtual QSize size();
        virtual const Image *read_frame();

    protected:
        Image _frame;
        QSize _size;
        bool _open;
    };
}
