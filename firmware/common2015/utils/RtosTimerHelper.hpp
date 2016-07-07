#pragma once

#include "rtos.h"
#include <functional>

/**
 * This class adds some nifty features to RtosTimer to make it more convenient
 * to use.
 *
 * RtosTimer allows you to pass a function pointer to the constructor that will
 * be called whenever the timer fires.  This class adds the ability to instead
 * pass an instance method and an object or a lambda.
 */
class RtosTimerHelper : public RtosTimer {
public:
    /// Call an instance method on a given object when the timer fires
    template <class T>
    RtosTimerHelper(T* instance, void (T::*method)(), os_timer_type type)
        : RtosTimer(&_timerFired, type, this) {
        _callback = std::bind(method, instance);
    }

    /// Call a function/lambda when the timer fires
    RtosTimerHelper(std::function<void()> callback, os_timer_type type)
        : RtosTimer(&_timerFired, type, this) {
        _callback = callback;
    }

    virtual ~RtosTimerHelper() {
        stop();
    }

    RtosTimerHelper(const RtosTimerHelper& other) = delete;

private:
    static void _timerFired(const void* instance) {
        RtosTimerHelper* thiss = const_cast<RtosTimerHelper*>(
            reinterpret_cast<const RtosTimerHelper*>(instance));
        thiss->_callback();
    }

    std::function<void()> _callback;
};
