#include "GamepadController.hpp"

using namespace std;

namespace {
    constexpr RJ::Time Dribble_Step_Time = 125 * 1000;
    constexpr RJ::Time Kicker_Step_Time = 125 * 1000;
    const float AXIS_MAX = 32768.0f;
    // cutoff for counting triggers as 'on'
    const float TRIGGER_CUTOFF = 0.9;
}

GamepadController::GamepadController()
    : _controller(nullptr), _lastDribblerTime(0), _lastKickerTime(0) {
    // initialize using the SDL joystick
    if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) {
        cerr << "ERROR: SDL could not initialize game controller system! SDL "
                "Error: " << SDL_GetError() << endl;
        return;
    }

    // Load extra controller mappings
    SDL_GameControllerAddMapping("030000006d04000016c2000011010000,Logitech F310 Gamepad (DInput)\
,platform:Linux,x:b0,a:b1,b:b2,y:b3,back:b8,start:b9,dpleft:h0.8,dpdown:h0.0,\
dpdown:h0.4,dpright:h0.0,dpright:h0.2,dpup:h0.0,dpup:h0.1,leftshoulder:h0.0,\
dpup:h0.1,leftshoulder:h0.0,leftshoulder:b4,lefttrigger:b6,rightshoulder:b5,\
righttrigger:b7,leftstick:b10,rightstick:b11,leftx:a0,lefty:a1,rightx:a2,righty:a3,");
    // SDL_GameControllerAddMapping("0300000057696920552047616d654300,Wii U GameCube Adapter Port 1,platform:Linux,x:b3,a:b0,b:b1,y:b2,start:b7,dpleft:b10,dpdown:b9,dpright:b11,dpup:b8,lefttrigger:a2,rightshoulder:b6,righttrigger:a5,leftx:a0,lefty:a1,rightx:a3,righty:a4,");

    // Controllers will be detected later if needed.
    connected = false;
    openJoystick();
}

GamepadController::~GamepadController() {
    QMutexLocker(&mutex());
    SDL_GameControllerClose(_controller);
    _controller = nullptr;
    SDL_Quit();
}

void GamepadController::openJoystick() {
    if (SDL_NumJoysticks()) {
        // Open the first available controller
        for (size_t i = 0; i < SDL_NumJoysticks(); ++i) {
            // setup the joystick as a game controller if available
            if (SDL_IsGameController(i)) {
                SDL_GameController* controller;
                controller = SDL_GameControllerOpen(i);
                connected = true;

                if (controller != nullptr) {
                    _controller = controller;
                    cout << "Using " << SDL_GameControllerName(_controller)
                         << " game controller" << endl;
                    break;
                } else {
                    cerr << "ERROR: Could not open controller! SDL Error: "
                         << SDL_GetError() << endl;
                }
                // Only support one joystick for now.
                return;
            }
        }
    }
}

void GamepadController::closeJoystick() {
    cout << "Closing " << SDL_GameControllerName(_controller) << endl;
    SDL_GameControllerClose(_controller);
    connected = false;
}

bool GamepadController::valid() const { return _controller != nullptr; }

void GamepadController::update() {
    QMutexLocker(&mutex());
    SDL_GameControllerUpdate();

    RJ::Time now = RJ::timestamp();

    if (connected) {
        // Check if dc
        if (!SDL_GameControllerGetAttached(_controller)) {
            closeJoystick();
            return;
        }
    } else {
        // Check if new controller found
        // TODO use the SDL event API to only run this if we receive a connected event.
        openJoystick();
        if (!connected) {
            return;
        }
    }

    /*
     *  DRIBBLER ON/OFF
     */
    if (SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_LEFTSHOULDER)) {
        _controls.dribble = false;
    } else if (SDL_GameControllerGetAxis(_controller, SDL_CONTROLLER_AXIS_TRIGGERLEFT) / AXIS_MAX > TRIGGER_CUTOFF) {
        _controls.dribble = true;
    }

    /*
     *  DRIBBLER POWER
     */
    if (SDL_GameControllerGetButton(_controller,
                                    SDL_CONTROLLER_BUTTON_A)) {
        if ((now - _lastDribblerTime) >= Dribble_Step_Time) {
            _controls.dribblerPower = max(_controls.dribblerPower - 0.1, 0.0);
            _lastDribblerTime = now;
        }
    } else if (SDL_GameControllerGetButton(_controller,
                                           SDL_CONTROLLER_BUTTON_Y)) {
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
    if (SDL_GameControllerGetButton(_controller,
                                    SDL_CONTROLLER_BUTTON_X)) {
        if ((now - _lastKickerTime) >= Kicker_Step_Time) {
            _controls.kickPower = max(_controls.kickPower - 0.1, 0.0);
            _lastKickerTime = now;
        }
    } else if (SDL_GameControllerGetButton(
                   _controller, SDL_CONTROLLER_BUTTON_B)) {
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
    _controls.kick = SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER);

    /*
     *  CHIP TRUE/FALSE
     */
    _controls.chip = SDL_GameControllerGetAxis(_controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT) / AXIS_MAX > TRIGGER_CUTOFF;

    /*
     *  VELOCITY ROTATION
     */
    // Logitech F310 Controller
    _controls.rotation =
        -1 * SDL_GameControllerGetAxis(_controller, SDL_CONTROLLER_AXIS_LEFTX) /
        AXIS_MAX;

    /*
     *  VELOCITY TRANSLATION
     */
    auto rightX =
        SDL_GameControllerGetAxis(_controller, SDL_CONTROLLER_AXIS_RIGHTX) /
        AXIS_MAX;
    auto rightY =
        -SDL_GameControllerGetAxis(_controller, SDL_CONTROLLER_AXIS_RIGHTY) /
        AXIS_MAX;

    Geometry2d::Point input(rightX, rightY);

    // Align along an axis using the DPAD as modifier buttons
    if (SDL_GameControllerGetButton(_controller,
                                    SDL_CONTROLLER_BUTTON_DPAD_DOWN)) {
        input.y() = -fabs(rightY);
        input.x() = 0;
    } else if (SDL_GameControllerGetButton(_controller,
                                           SDL_CONTROLLER_BUTTON_DPAD_UP)) {
        input.y() = fabs(rightY);
        input.x() = 0;
    } else if (SDL_GameControllerGetButton(_controller,
                                           SDL_CONTROLLER_BUTTON_DPAD_LEFT)) {
        input.y() = 0;
        input.x() = -fabs(rightX);
    } else if (SDL_GameControllerGetButton(_controller,
                                           SDL_CONTROLLER_BUTTON_DPAD_RIGHT)) {
        input.y() = 0;
        input.x() = fabs(rightX);
    }

    // Floating point precision error rounding
    if (_controls.kickPower < 1e-1) _controls.kickPower = 0;
    if (_controls.dribblerPower < 1e-1) _controls.dribblerPower = 0;
    if (fabs(_controls.rotation) < 5e-2) _controls.rotation = 0;
    if (fabs(input.y()) < 5e-2) input.y() = 0;
    if (fabs(input.x()) < 5e-2) input.x() = 0;

    _controls.translation = Geometry2d::Point(input.x(), input.y());
}

JoystickControlValues GamepadController::getJoystickControlValues() {
    QMutexLocker(&mutex());
    return _controls;
}

void GamepadController::reset() {
    QMutexLocker(&mutex());
    _controls.dribble = false;
}
