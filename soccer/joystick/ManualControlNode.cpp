#include "ManualControlNode.hpp"
#include "GamepadController.hpp"
#include "InputDeviceManager.hpp"


ManualControlNode::ManualControlNode(Context* context) : _context(context) {
    // initialise device manager
    initSDL(_context->is_joystick_controlled);
}


ManualControlNode::run() {
    // TODO: update input devices
    update();
    // TODO: set which robots are joystick controlled
    // TODO: set intents for the robots
}


void ManualControlNode::initSDL(
    std::array<bool, Num_Shells> _is_joystick_controlled) {
    //TODO: initialize array of input devices
    _inputDevices.reserve(Num_Shells);
    
    //TODO: initialize SDL system
    // initialize SDL event system
    if (SDL_Init(SDL_INIT_EVENT) != 0) {
        cerr << "ERROR: SDL could not initialize event system! SDL Error: "
            << SDL_GetError() << endl;
        return;
    }

    GamepadController::initDeviceType();
}

ManualControlNode::update() {
    //TODO: get next event in eventqueue
    //TODO: check for new controllers
    //TODO: check removal of controller
    //TODO: map controller and SDL indices
    //TODO: update controller movement/button
    //TODO: update contexts
}
