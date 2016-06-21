#include "GamepadController.hpp"

using namespace std;

namespace {
constexpr RJ::Time Dribble_Step_Time = 125 * 1000;
constexpr RJ::Time Kicker_Step_Time = 125 * 1000;
const float AXIS_MAX = 32768.0f;
}

GamepadController::GamepadController()
    : _controller(nullptr), _lastDribblerTime(0), _lastKickerTime(0) {

    if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) {
        cerr << "ERROR: SDL could not initialize! SDL Error: " << SDL_GetError()
             << endl;
        return;
    }

    if (SDL_NumJoysticks()) {
        // Open the first available controller
        for (size_t i = 0; i < SDL_NumJoysticks(); ++i) {
            if (SDL_IsGameController(i)) {
                SDL_GameController* controller;

                cerr << "controller '" << i << "' is compatible, named '" << SDL_GameControllerNameForIndex(i) << "'" << endl;
                controller = SDL_GameControllerOpen(i);

                char* mapping;
                mapping = SDL_GameControllerMapping(controller);

                cerr << "controller " << i << " is mapped as '" << mapping << "'" << endl;
                SDL_free(mapping);

                if (controller != nullptr) {
                    _controller = controller;
                    cout << "Controller connected to " << SDL_GameControllerName(_controller) << endl;

                    SDL_Joystick* joy;
                    joy = SDL_GameControllerGetJoystick(_controller);

                    SDL_JoystickGUID guid = SDL_JoystickGetGUID(joy);

                    string guid_str(' ', 34);
                    SDL_JoystickGetGUIDString(guid, guid_str.data(), guid_str.length());
                    
                    // "GUID,name,mapping"
                    // Example: "341a3608000000000000504944564944,
                    //           Afterglow PS3 Controller,
                    //           a:b1,b:b2,y:b3,x:b0,start:b9,guide:b12,back:b8,dpup:h0.1,dpleft:h0.8,dpdown:h0.4,dpright:h0.2,leftshoulder:b4,rightshoulder:b5,leftstick:b10,rightstick:b11,leftx:a0,lefty:a1,rightx:a2,righty:a3,lefttrigger:b6,righttrigger:b7"
                    string map_str(guid_str);
                    map_str += "," + SDL_JoystickName(joy);
                    map_str += ",a:b1";
                    map_str += ",b:b2";
                    map_str += ",y:b3";
                    map_str += ",x:b0";
                    map_str += ",start:b9";
                    SDL_GameControllerAddMapping(map_str.c_str());

                    break;
                } else {
                    cerr << "ERROR: Could not open controller! SDL Error: " << SDL_GetError()
                     << endl;
                }
            }
        }
    } else {
        cout << "WARNING: No manual controllers connected" << endl;
    }
}

GamepadController::~GamepadController() {
    QMutexLocker(&mutex());
    SDL_GameControllerClose(_controller);
    _controller = nullptr;
    SDL_Quit();
}

bool GamepadController::valid() const { return _controller != nullptr; }

void GamepadController::update() {
    QMutexLocker(&mutex());
    SDL_GameControllerUpdate();

    RJ::Time now = RJ::timestamp();

    /*
     *  DRIBBLER ON/OFF
     */
    if (SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_Y)) {
        _controls.dribble = !_controls.dribble;
    }

    /*
     *  DRIBBLER POWER
     */
    if (SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_LEFTSTICK)) {
        if ((now - _lastDribblerTime) >= Dribble_Step_Time) {
            _controls.dribblerPower = max(_controls.dribblerPower - 0.1, 0.0);
            _lastDribblerTime = now;
        }
    } else if (SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_RIGHTSTICK)) {
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
    if (SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_LEFTSHOULDER)) {
        if ((now - _lastKickerTime) >= Kicker_Step_Time) {
            _controls.kickPower = max(_controls.kickPower - 0.1, 0.0);
            _lastKickerTime = now;
        }
    } else if (SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER)) {
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
    _controls.kick = SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_A);

    /*
     *  CHIP TRUE/FALSE
     */
    _controls.chip = SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_B);

    /*
     *  VELOCITY ROTATION
     */
    // Logitech F310 Controller
    _controls.rotation = -1 * SDL_GameControllerGetAxis(_controller, SDL_CONTROLLER_AXIS_LEFTX) / AXIS_MAX;

    /*
     *  VELOCITY TRANSLATION
     */
    auto rightX = SDL_GameControllerGetAxis(_controller, SDL_CONTROLLER_AXIS_RIGHTX) / AXIS_MAX;
    auto rightY = -SDL_GameControllerGetAxis(_controller, SDL_CONTROLLER_AXIS_RIGHTY) / AXIS_MAX;

    Geometry2d::Point input(rightX, rightY);

    // Align along an axis using the DPAD as modifier buttons
    auto mVal = fabs(rightY);
    if (SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_DPAD_DOWN)) {
        input.y() = mVal;
        input.x() = 0;
    } else if (SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_DPAD_UP)) {
        input.y() = -mVal;
        input.x() = 0;
    } else if (SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_DPAD_RIGHT)) {
        input.y() = 0;
        input.x() = mVal;
    } else if (SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_DPAD_LEFT)) {
        input.y() = 0;
        input.x() = -mVal;
    }

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
