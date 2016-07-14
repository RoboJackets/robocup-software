#include "GamepadController.hpp"

using namespace std;

namespace {
constexpr RJ::Time Dribble_Step_Time = 125 * 1000;
constexpr RJ::Time Kicker_Step_Time = 125 * 1000;
const float AXIS_MAX = 32768.0f;
}

GamepadController::GamepadController()
    : _controller(nullptr), _lastDribblerTime(0), _lastKickerTime(0) {

    // initialize using the SDL joystick
    if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) {
        cerr << "ERROR: SDL could not initialize game controller system! SDL Error: " << SDL_GetError()
             << endl;
        return;
    }

    if (SDL_NumJoysticks()) {
        // Open the first available controller
        for (size_t i = 0; i < SDL_NumJoysticks(); ++i) {
            // // open joystick, print out info about it, then close it back
            // SDL_Joystick *joy;
            // joy = SDL_JoystickOpen(i);
            // if (joy) {
            //     cerr << "Joystick " << i << ":" << endl;
            //     cerr << "  Name:\t\t\t" << SDL_JoystickNameForIndex(i) << endl;
            //     cerr << "  Number of Axes:\t" << SDL_JoystickNumAxes(joy) << endl;
            //     cerr << "  Number of Btns:\t" << SDL_JoystickNumButtons(joy) << endl;
            //     cerr << "  Number of Balls:\t" << SDL_JoystickNumBalls(joy) << endl;
            //     cerr << "  Game Controller:\t" << static_cast<bool>(SDL_IsGameController(i)) << endl;
            // } else {
            //     cerr << "Unable to open joystick number " << i << endl;
            // }

            // // close the joystick back
            // if (SDL_JoystickGetAttached(joy)) {
            //     SDL_JoystickClose(joy);
            // }

            // setup the joystick as a game controller if available
            if (SDL_IsGameController(i)) {
                SDL_GameController* controller;
                controller = SDL_GameControllerOpen(i);

                if (controller != nullptr) {
                    _controller = controller;
                    cout << "Using " << SDL_GameControllerName(_controller) << " game controller" << endl;
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
    } else if (SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_Y)) {
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
    _controls.chip = SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_X);

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
    if (SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_DPAD_DOWN)) {
        input.y() = -fabs(rightY);
        input.x() = 0;
    } else if (SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_DPAD_UP)) {
        input.y() = fabs(rightY);
        input.x() = 0;
    } else if (SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_DPAD_LEFT)) {
        input.y() = 0;
        input.x() = -fabs(rightX);
    } else if (SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_DPAD_RIGHT)) {
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
