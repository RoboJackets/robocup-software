#pragma once

#include "mbed.h"
#include "rtos.h"

// Helper function for using an rtos mail queue within a class

template <class T, unsigned int size>
class MailHelper {
public:
    MailHelper() {
#ifdef CMSIS_OS_RTX
        memset(_q, 0, sizeof(_q));
        _ptr[0] = _q;

        memset(_m, 0, sizeof(_m));
        _ptr[1] = _m;

        _def.pool = _ptr;
        _def.item_sz = sizeof(T);
        _def.queue_sz = size;
#endif
    };

    osMailQDef_t* def() { return &_def; }

private:
    osMailQDef_t _def;

#ifdef CMSIS_OS_RTX
    uint32_t _q[4 + size];
    uint32_t _m[3 + ((sizeof(T) + 3) / 4) * size];
    void* _ptr[2];
#endif
};
