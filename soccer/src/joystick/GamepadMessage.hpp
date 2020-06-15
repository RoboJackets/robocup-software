#pragma once
#include <rj_common/time.hpp>

namespace joystick {

struct AnalogStickState {
    int16_t x;
    int16_t y;
};

struct SticksState {
    AnalogStickState left{};
    AnalogStickState right{};
};

struct TriggersState {
    int16_t left;
    int16_t right;
};

struct DPadState {
    bool up;
    bool down;
    bool left;
    bool right;
};

/**
 * Struct representing the buttons
 */
struct ButtonsState {
    bool a;
    bool b;
    bool x;
    bool y;
    bool back;
    bool guide;
    bool start;
    bool left_stick;
    bool right_stick;
    bool left_shoulder;
    bool right_shoulder;
    bool max;
};

/**
 * Struct representing the states of a SDL gamepad
 */
struct GamepadMessage {
    /** \brief Unique ID for the gamepad */
    int unique_id{};

    /** \brief Time of last update */
    RJ::Time update_time{};
    SticksState sticks{};
    TriggersState triggers{};
    ButtonsState buttons{};
    DPadState dpad{};
};
}  // namespace joystick
