#include "GamepadJoystick.hpp"

using namespace Packet;
using namespace std;

static constexpr auto Dribble_Step_Time = RJ::Seconds(0.125);
static constexpr auto Kicker_Step_Time = RJ::Seconds(0.125);

static constexpr float AXIS_MAX = 32768.0f;

GamepadJoystick::GamepadJoystick()
    : _joystick(nullptr), _lastDribblerTime(), _lastKickerTime() {
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
        cerr << "SDL could not initialize! SDL Error: " << SDL_GetError()
             << endl;
        return;
    }
    if (SDL_NumJoysticks() < 1) {
        cout << "Warning: No joysticks connected!" << endl;
    } else {
        _joystick = SDL_JoystickOpen(0);
        if (_joystick == nullptr) {
            cerr << "SDL could not open joystick! SDL Error: " << SDL_GetError()
                 << endl;
        } else {
            cout << "Joystick connected to " << SDL_JoystickName(0) << endl;
        }
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

    // DRIBBLER CONTROL
    if (SDL_JoystickGetButton(_joystick, 6)) {
        _controls.dribble = false;
    } else if (SDL_JoystickGetButton(_joystick, 4)) {
        _controls.dribble = true;
    }

    // DRIBBLER POWER CONTROL
    const auto now = RJ::now();
    if (SDL_JoystickGetButton(_joystick, 1)) {
        if ((now - _lastDribblerTime) >= Dribble_Step_Time) {
            _controls.dribblerPower = max(_controls.dribblerPower - 0.1, 0.0);
            _lastDribblerTime = now;
        }
    } else if (SDL_JoystickGetButton(_joystick, 3)) {
        if ((now - _lastDribblerTime) >= Dribble_Step_Time) {
            _controls.dribblerPower = min(_controls.dribblerPower + 0.1, 1.0);
            _lastDribblerTime = now;
        }
    } else {
        // Let dribbler speed change immediately
        _lastDribblerTime = now - Dribble_Step_Time;
    }

    // KICKER POWER CONTROL
    if (SDL_JoystickGetButton(_joystick, 0)) {
        if ((now - _lastKickerTime) >= Kicker_Step_Time) {
            _controls.kickPower = max(_controls.kickPower - 0.1, 0.0);
            _lastKickerTime = now;
        }
    } else if (SDL_JoystickGetButton(_joystick, 2)) {
        if ((now - _lastKickerTime) >= Kicker_Step_Time) {
            _controls.kickPower = min(_controls.kickPower + 0.1, 1.0);
            _lastKickerTime = now;
        }
    } else {
        _lastKickerTime = now - Kicker_Step_Time;
    }

    // Kicking is triggered by a chip as well. If you only want a chip, remove
    // 5.
    _controls.kick = SDL_JoystickGetButton(_joystick, 7) |
                     SDL_JoystickGetButton(_joystick, 5);

    _controls.chip = SDL_JoystickGetButton(_joystick, 5);

    // for(int i = 0; i < SDL_JoystickNumButtons(_joystick); i++)
    //    cout << static_cast<int>(SDL_JoystickGetButton(_joystick, i));
    // cout << endl;

    // Rotation
    auto rightX = SDL_JoystickGetAxis(_joystick, 2) / AXIS_MAX; 
    // Move L/R
    auto leftX = SDL_JoystickGetAxis(_joystick, 0) / AXIS_MAX;
    // Move U/D
    auto leftY = -SDL_JoystickGetAxis(_joystick, 1) / AXIS_MAX;

    Geometry2d::Point input(leftX, leftY);

    auto mVal = fabs(leftY);

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

    _controls.rotation = -rightX;
}

JoystickControlValues GamepadJoystick::getJoystickControlValues() {
    QMutexLocker(&mutex());
    return _controls;
}

void GamepadJoystick::reset() {
    QMutexLocker(&mutex());
    _controls.dribble = false;
}
