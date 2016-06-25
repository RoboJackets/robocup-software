#include "GamepadXbox.hpp"

using namespace std;

namespace {
constexpr RJ::Time Dribble_Step_Time = 85 * 1000;
constexpr RJ::Time Kicker_Step_Time = 85 * 1000;
}

GamepadXbox::GamepadXbox()
    : _id(GAMEPAD_COUNT), _lastDribblerTime(0), _lastKickerTime(0) {

    // setup the gamepad initialization
    GamepadInit();

    for (size_t i = 0; i < GAMEPAD_COUNT; ++i) {
        GAMEPAD_DEVICE dev = static_cast<GAMEPAD_DEVICE>(i);

        if (GamepadIsConnected(dev)) {
            _id = dev;
            break;
        }
    }

    if (valid()) {
        cout << "INFO: Using Xbox controller at index " << _id << "." << endl;
    } else {
        cout << "WARNING: No Xbox 360 controllers detected." << endl;
    }
}

GamepadXbox::~GamepadXbox() {
    QMutexLocker(&mutex());
    GamepadShutdown();
}

bool GamepadXbox::valid() const { return _id != GAMEPAD_COUNT; }

void GamepadXbox::update() {
    QMutexLocker(&mutex());
    GamepadUpdate();

    RJ::Time now = RJ::timestamp();

    /*
     *  DRIBBLER POWER
     */
    if (GamepadButtonDown(_id, BUTTON_LEFT_SHOULDER)) {
        // if holding down the left shoulder button, let the dribbler's velocity
        // be adjusted using the DPAD
        if (GamepadButtonDown(_id, BUTTON_A)) {
            if ((now - _lastDribblerTime) >= Dribble_Step_Time) {
                _controls.dribblerPower = max(_controls.dribblerPower - 0.1, 0.0);
                _lastDribblerTime = now;
            }
        } else if (GamepadButtonDown(_id, BUTTON_Y)) {
            if ((now - _lastDribblerTime) >= Dribble_Step_Time) {
                _controls.dribblerPower = min(_controls.dribblerPower + 0.1, 1.0);
                _lastDribblerTime = now;
            }
        }
    } else if (GamepadButtonDown(_id, BUTTON_Y) && !(GamepadButtonDown(_id, BUTTON_LEFT_SHOULDER)) && !(GamepadButtonDown(_id, BUTTON_RIGHT_SHOULDER))) {
        /*
         *  DRIBBLER ON/OFF
         */
        if ((now - _lastDribblerTime) >= Dribble_Step_Time) {
            _controls.dribble = !_controls.dribble;
            _lastDribblerTime = now;
        }
    } else {
        // Let dribbler speed change immediately
        _lastDribblerTime = now - Dribble_Step_Time;
    }

    /*
     *  KICKER POWER
     */
    if (GamepadButtonDown(_id, BUTTON_RIGHT_SHOULDER)) {
        // if holding down the right shoulder button, let the kicker's power
        // be adjusted using the DPAD
        if (GamepadButtonDown(_id, BUTTON_A)) {
            if ((now - _lastKickerTime) >= Kicker_Step_Time) {
                _controls.kickPower = max(_controls.kickPower - 0.1, 0.0);
                _lastKickerTime = now;
            }
        } else if (GamepadButtonDown(_id, BUTTON_Y)) {
            if ((now - _lastKickerTime) >= Kicker_Step_Time) {
                _controls.kickPower = min(_controls.kickPower + 0.1, 1.0);
                _lastKickerTime = now;
            }
        }
    } else {
        _lastKickerTime = now - Kicker_Step_Time;
    }


    if (!(GamepadButtonDown(_id, BUTTON_LEFT_SHOULDER)) && !(GamepadButtonDown(_id, BUTTON_RIGHT_SHOULDER))) {
        /*
         *  KICK TRUE/FALSE
         */
        _controls.kick = GamepadButtonDown(_id, BUTTON_A);

        /*
         *  CHIP TRUE/FALSE
         */
        _controls.chip = GamepadButtonDown(_id, BUTTON_X);

    }

    /*
     *  VELOCITY ROTATION
     */
    float rotation_x = 0.0, rotation_y = 0.0;
    GamepadStickNormXY(_id, STICK_RIGHT, &rotation_x, &rotation_y);
    _controls.rotation = -rotation_x;

    /*
     *  VELOCITY TRANSLATION
     */
    float vel_x = 0.0, vel_y = 0.0;
    GamepadStickNormXY(_id, STICK_LEFT, &vel_x, &vel_y);

    // use the left and right triggers to be used as an added
    // x velocity component
    float right_trigger = GamepadTriggerLength(_id, TRIGGER_LEFT);
    float left_trigger = GamepadTriggerLength(_id, TRIGGER_RIGHT);
    float x_vel_skew = left_trigger + right_trigger;

    vel_x = vel_x + x_vel_skew;

    Geometry2d::Point input(vel_x, vel_y);

    // Use the DPAD for constant axis movement, but only if not adjusting
    // the dribbler velocity or kicker power. this is useful for testing motion control things
    if (!(GamepadButtonDown(_id, BUTTON_LEFT_SHOULDER)) && !(GamepadButtonDown(_id, BUTTON_RIGHT_SHOULDER))) {
        if (GamepadButtonDown(_id, BUTTON_DPAD_UP)) {
            input.y() = 0.5;
            input.x() = 0;
        } else if (GamepadButtonDown(_id, BUTTON_DPAD_DOWN)) {
            input.y() = -0.5;
            input.x() = 0;
        } else if (GamepadButtonDown(_id, BUTTON_DPAD_LEFT)) {
            input.y() = 0;
            input.x() = -0.5;
        } else if (GamepadButtonDown(_id, BUTTON_DPAD_RIGHT)) {
            input.y() = 0;
            input.x() = 0.5;
        }
    }

    // Floating point precision error rounding
    if (_controls.kickPower < 1e-1) _controls.kickPower = 0;
    if (_controls.dribblerPower < 1e-1) _controls.dribblerPower = 0;
    if (fabs(_controls.rotation) < 0.5e-2) _controls.rotation = 0;
    if (fabs(input.y()) < 0.5e-2) input.y() = 0;
    if (fabs(input.x()) < 0.5e-2) input.x() = 0;

    _controls.translation = Geometry2d::Point(input.x(), input.y());
}

JoystickControlValues GamepadXbox::getJoystickControlValues() {
    QMutexLocker(&mutex());
    return _controls;
}

void GamepadXbox::reset() {
    QMutexLocker(&mutex());
    _controls.dribble = false;
}
