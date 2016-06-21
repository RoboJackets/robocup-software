#include "GamepadJoystick.hpp"

using namespace std;

namespace {
constexpr RJ::Time Dribble_Step_Time = 125 * 1000;
constexpr RJ::Time Kicker_Step_Time = 125 * 1000;
const float AXIS_MAX = 32768.0f;
}

GamepadJoystick::GamepadJoystick()
    : _joystick(nullptr), _lastDribblerTime(0), _lastKickerTime(0) {

    if (SDL_Init(SDL_INIT_JOYSTICK) != 0) {
        cerr << "ERROR: SDL could not initialize! SDL Error: " << SDL_GetError()
             << endl;
        return;
    }

    if (SDL_NumJoysticks()) {
        _joystick = SDL_JoystickOpen(0);

        if (_joystick != nullptr) {
            cout << "Joystick connected to " << SDL_JoystickName(0) << endl;
        } else {
            cerr << "ERROR: Could not open joystick! SDL Error: " << SDL_GetError()
                 << endl;
        }
    } else {
        cout << "WARNING: No joysticks connected!" << endl;
    }
}

GamepadJoystick::~GamepadJoystick() {
    QMutexLocker(&mutex());
    SDL_JoystickClose(_joystick);
    _joystick = nullptr;
    SDL_Quit();
}

bool GamepadJoystick::valid() const { return _joystick != nullptr; }

void GamepadJoystick::update() {
    QMutexLocker(&mutex());
    SDL_JoystickUpdate();

    RJ::Time now = RJ::timestamp();

    /*
     *  DRIBBLER ON/OFF
     */
    if (SDL_JoystickGetButton(_joystick, 6)) {
        _controls.dribble = !_controls.dribble;
    }

    /*
     *  DRIBBLER POWER
     */
    // Logitech_Dual_Action Controller
    // if (SDL_JoystickGetButton(_joystick, 1)) {
    //
    // Logitech F310 Controller & Xbox 360 Controller
    if (SDL_JoystickGetButton(_joystick, 0)) {

        if ((now - _lastDribblerTime) >= Dribble_Step_Time) {
            _controls.dribblerPower = max(_controls.dribblerPower - 0.1, 0.0);
            _lastDribblerTime = now;
        }

    // Logitech F310 Controller & Logitech_Dual_Action Controller & Xbox 360 Controller
    } else if (SDL_JoystickGetButton(_joystick, 3)) {

        if ((now - _lastDribblerTime) >= Dribble_Step_Time) {
            _controls.dribblerPower = min(_controls.dribblerPower + 0.1, 1.0);
            _lastDribblerTime = now;
        }
    } else {
        // Let dribbler speed change immediately
        _lastDribblerTime = now - Dribble_Step_Time;
    }

    /*
     *  KICKER POWER
     */
    // Logitech_Dual_Action Controller
    // if (SDL_JoystickGetButton(_joystick, 0)) {
    //
    // Logitech F310 Controller & Xbox 360 Controller
    if (SDL_JoystickGetButton(_joystick, 2)) {

        if ((now - _lastKickerTime) >= Kicker_Step_Time) {
            _controls.kickPower = max(_controls.kickPower - 0.1, 0.0);
            _lastKickerTime = now;
        }
    // Logitech_Dual_Action Controller
    // } else if (SDL_JoystickGetButton(_joystick, 2)) {
    //
    // Logitech F310 Controller & Xbox 360 Controller
    } else if (SDL_JoystickGetButton(_joystick, 1)) {

        if ((now - _lastKickerTime) >= Kicker_Step_Time) {
            _controls.kickPower = min(_controls.kickPower + 0.1, 1.0);
            _lastKickerTime = now;
        }
    } else {
        _lastKickerTime = now - Kicker_Step_Time;
    }

    /*
     *  KICK TRUE/FALSE
     */
    _controls.kick = SDL_JoystickGetButton(_joystick, 7);

    /*
     *  CHIP TRUE/FALSE
     */
    _controls.chip = SDL_JoystickGetButton(_joystick, 5);

    /*
     *  VELOCITY ROTATION
     */
    // Logitech_Dual_Action Controller & Xbox 360 Controller
    // auto leftX = SDL_JoystickGetAxis(_joystick, 0) / AXIS_MAX;
    //
    // Logitech F310 Controller
    auto leftX = SDL_JoystickGetAxis(_joystick, 6) / AXIS_MAX;
    _controls.rotation = -leftX;

    /*
     *  VELOCITY TRANSLATION
     */
    // Logitech_Dual_Action Controller
    // auto rightX = SDL_JoystickGetAxis(_joystick, 2) / AXIS_MAX;
    // auto rightY = -SDL_JoystickGetAxis(_joystick, 3) / AXIS_MAX;
    //
    // Logitech F310 Controller & Xbox 360 Controller
    auto rightX = SDL_JoystickGetAxis(_joystick, 2) / AXIS_MAX;
    auto rightY = -SDL_JoystickGetAxis(_joystick, 4) / AXIS_MAX;

    Geometry2d::Point input(rightX, rightY);

    auto mVal = fabs(rightY);

    if (SDL_JoystickGetAxis(_joystick, 5) < 0) {
        input.y() = mVal;
        input.x() = 0;
    } else if (SDL_JoystickGetAxis(_joystick, 5) > 0) {
        input.y() = -mVal;
        input.x() = 0;
    } else if (SDL_JoystickGetAxis(_joystick, 4) > 0) {
        input.y() = 0;
        input.x() = mVal;
    } else if (SDL_JoystickGetAxis(_joystick, 4) < 0) {
        input.y() = 0;
        input.x() = -mVal;
    }

    _controls.translation = Geometry2d::Point(input.x(), input.y());
}

JoystickControlValues GamepadJoystick::getJoystickControlValues() {
    QMutexLocker(&mutex());
    return _controls;
}

void GamepadJoystick::reset() {
    QMutexLocker(&mutex());
    _controls.dribble = false;
}
