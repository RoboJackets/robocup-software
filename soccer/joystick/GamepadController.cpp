#include "GamepadController.hpp"
#include <algorithm>

using namespace std;

namespace {
constexpr auto Dribble_Step_Time = RJ::Seconds(0.125);
constexpr auto Kicker_Step_Time = RJ::Seconds(0.125);
const float AXIS_MAX = 32768.0f;
// cutoff for counting triggers as 'on'
const float TRIGGER_CUTOFF = 0.9;
}  // namespace

std::vector<int> GamepadController::controllersInUse = {};
int GamepadController::joystickRemoved = -1;

GamepadController::GamepadController()
    : _controller(nullptr), _lastDribblerTime(), _lastKickerTime() {
    // initialize using the SDL joystick
    if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) {
        cerr << "ERROR: SDL could not initialize game controller system! SDL "
                "Error: " << SDL_GetError() << endl;
        return;
    }

    // Attempt to add additional mappings (relative to run)
    if (SDL_GameControllerAddMappingsFromFile(
            ApplicationRunDirectory()
                .filePath("../external/sdlcontrollerdb/gamecontrollerdb.txt")
                .toStdString()
                .c_str()) == -1) {
        cout << "Failed adding additional SDL Gamecontroller Mappings: "
             << SDL_GetError() << endl;
    }

    // Controllers will be detected later if needed.
    connected = false;
    controllerId = -1;
    robotId = -1;
    openJoystick();
}

GamepadController::~GamepadController() {
    QMutexLocker(&mutex());
    SDL_GameControllerClose(_controller);
    _controller = nullptr;
    SDL_Quit();
}

void GamepadController::openJoystick() {
    if (SDL_NumJoysticks() != 0) {
        // Open the first available controller
        for (int i = 0; i < SDL_NumJoysticks(); ++i) {
            // setup the joystick as a game controller if available
            if (std::find(controllersInUse.begin(), controllersInUse.end(),
                          i) == controllersInUse.end() &&
                SDL_IsGameController(i)) {
                SDL_GameController* controller;
                controller = SDL_GameControllerOpen(i);

                if (controller != nullptr) {
                    _controller = controller;
                    connected = true;
                    controllerId = i;
                    controllersInUse.push_back(i);
                    sort(controllersInUse.begin(), controllersInUse.end());
                    cout << "Using " << SDL_GameControllerName(_controller)
                         << " game controller as controller # "
                         << controllersInUse.size() << endl;
                    break;
                }
                cerr << "ERROR: Could not open controller! SDL Error: "
                     << SDL_GetError() << endl;

                return;
            }
        }
    }
}

void GamepadController::closeJoystick() {
    cout << "Closing " << SDL_GameControllerName(_controller) << endl;
    SDL_GameControllerClose(_controller);
    auto index =
        find(controllersInUse.begin(), controllersInUse.end(), controllerId);
    if (index != controllersInUse.end()) {
        for (auto i = index + 1; i != controllersInUse.end(); i++) {
            *i -= 1;
        }
        controllersInUse.erase(index);
    }
    joystickRemoved = controllerId;
    controllerId = -1;

    robotId = -1;
    connected = false;

    // Clear events from queue
    SDL_Event event;
    while (SDL_PollEvent(&event) != 0) {
    }
}

bool GamepadController::valid() const { return connected; }

void GamepadController::update() {
    SDL_GameControllerUpdate();

    RJ::Time now = RJ::now();

    if (connected) {
        // Check if dc
        if (joystickRemoved >= 0 && controllerId > joystickRemoved) {
            controllerId -= 1;
        }
        if (SDL_GameControllerGetAttached(_controller) == 0u) {
            closeJoystick();
            return;
        }
    } else {
        // Check if new controller found
        // TODO use the SDL event API to only run this if we receive a connected
        // event.
        openJoystick();
        if (!connected) {
            return;
        }
    }

    // Don't do anything until we have an event
    // TODO stop abusing the queue here and use the event api.
    if (SDL_HasEvents(SDL_CONTROLLERAXISMOTION, SDL_CONTROLLERBUTTONUP) == 0u) {
        return;
    }

    /*
     *  DRIBBLER ON/OFF
     */
    if (SDL_GameControllerGetButton(_controller,
                                    SDL_CONTROLLER_BUTTON_LEFTSHOULDER) != 0u) {
        _controls.dribble = true;
    } else if (SDL_GameControllerGetAxis(_controller,
                                         SDL_CONTROLLER_AXIS_TRIGGERLEFT) /
                   AXIS_MAX >
               TRIGGER_CUTOFF) {
        _controls.dribble = false;
    }

    /*
     *  DRIBBLER POWER
     */
    if (SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_A) !=
        0u) {
        if ((now - _lastDribblerTime) >= Dribble_Step_Time) {
            _controls.dribblerPower = max(_controls.dribblerPower - 0.1, 0.0);
            _lastDribblerTime = now;
        }
    } else if (SDL_GameControllerGetButton(_controller,
                                           SDL_CONTROLLER_BUTTON_Y) != 0u) {
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
    if (SDL_GameControllerGetButton(_controller, SDL_CONTROLLER_BUTTON_X) !=
        0u) {
        if ((now - _lastKickerTime) >= Kicker_Step_Time) {
            _controls.kickPower = max(_controls.kickPower - 0.1, 0.0);
            _lastKickerTime = now;
        }
    } else if (SDL_GameControllerGetButton(_controller,
                                           SDL_CONTROLLER_BUTTON_B) != 0u) {
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
    _controls.kick =
        (SDL_GameControllerGetButton(
             _controller, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER) != 0u);

    /*
     *  CHIP TRUE/FALSE
     */
    _controls.chip = SDL_GameControllerGetAxis(
                         _controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT) /
                         AXIS_MAX >
                     TRIGGER_CUTOFF;

    /*
     *  VELOCITY ROTATION
     */
    // Logitech F310 Controller
    _controls.rotation = -1 * SDL_GameControllerGetAxis(
                                  _controller, SDL_CONTROLLER_AXIS_RIGHTX) /
                         AXIS_MAX;

    /*
     *  VELOCITY TRANSLATION
     */
    auto leftX =
        SDL_GameControllerGetAxis(_controller, SDL_CONTROLLER_AXIS_LEFTX) /
        AXIS_MAX;
    auto leftY =
        -SDL_GameControllerGetAxis(_controller, SDL_CONTROLLER_AXIS_LEFTY) /
        AXIS_MAX;

    Geometry2d::Point input(leftX, leftY);

    // Align along an axis using the DPAD as modifier buttons
    if (SDL_GameControllerGetButton(_controller,
                                    SDL_CONTROLLER_BUTTON_DPAD_DOWN) != 0u) {
        input.y() = -fabs(leftY);
        input.x() = 0;
    } else if (SDL_GameControllerGetButton(
                   _controller, SDL_CONTROLLER_BUTTON_DPAD_UP) != 0u) {
        input.y() = fabs(leftY);
        input.x() = 0;
    } else if (SDL_GameControllerGetButton(
                   _controller, SDL_CONTROLLER_BUTTON_DPAD_LEFT) != 0u) {
        input.y() = 0;
        input.x() = -fabs(leftX);
    } else if (SDL_GameControllerGetButton(
                   _controller, SDL_CONTROLLER_BUTTON_DPAD_RIGHT) != 0u) {
        input.y() = 0;
        input.x() = fabs(leftX);
    }

    // Floating point precision error rounding
    if (_controls.kickPower < 1e-1) {
        _controls.kickPower = 0;
    }
    if (_controls.dribblerPower < 1e-1) {
        _controls.dribblerPower = 0;
    }
    if (fabs(_controls.rotation) < 5e-2) {
        _controls.rotation = 0;
    }
    if (fabs(input.y()) < 5e-2) {
        input.y() = 0;
    }
    if (fabs(input.x()) < 5e-2) {
        input.x() = 0;
    }

    _controls.translation = Geometry2d::Point(input.x(), input.y());
}

JoystickControlValues GamepadController::getJoystickControlValues() {
    return _controls;
}

void GamepadController::reset() {
    
    _controls.dribble = false;
}
